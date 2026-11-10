// Offline builder: native MoveIt joint state space + native OMPL PRM. No motion clients.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::string readText(const std::string& file)
{
  std::ifstream stream(file, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read " + file);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
template <class Message> Message readMessage(const std::string& file)
{
  const auto bytes = readText(file);
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw = serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  rclcpp::Serialization<Message> serialization;
  Message message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

int main(int argc, char** argv)
{
  if (argc != 5) return 2;
  const std::string folder = argv[1];
  const double build_seconds = std::stod(argv[2]);
  rclcpp::init(argc, argv);
  try
  {
    ompl::RNG::setSeed(42);
    auto node = std::make_shared<rclcpp::Node>("offline_left_arm_prm_builder");
    robot_model_loader::RobotModelLoader::Options options(readText(folder + "/robot.urdf"),
                                                        readText(folder + "/robot.srdf"));
    options.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoader loader(node, options);
    auto model = loader.getModel();
    if (!model) throw std::runtime_error("Cannot load robot model");
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setPlanningSceneMsg(readMessage<moveit_msgs::msg::PlanningScene>(folder + "/scene.cdr"));
    auto request = readMessage<moveit_msgs::msg::MotionPlanRequest>(folder + "/request.cdr");
    request.planner_id = "PRM";
    planning_interface::PlannerConfigurationSettings group;
    group.name = group.group = "left_arm";
    group.config = {{"longest_valid_segment_fraction", argv[4]}};
    auto specific = group;
    specific.name = "left_arm[PRM]";
    specific.config["type"] = "geometric::PRM";
    specific.config["max_nearest_neighbors"] = argv[3];
    planning_interface::PlannerConfigurationMap configs{{group.name, group}, {specific.name, specific}};
    ompl_interface::OMPLInterface interface(model, configs, node, "ompl");
    moveit_msgs::msg::MoveItErrorCodes code;
    auto context = interface.getPlanningContext(scene, request, code);
    if (!context) throw std::runtime_error("Cannot create native PRM context: " + std::to_string(code.val));
    // MoveIt starts/stops its lazy joint-goal sampler in this public solve entry.
    planning_interface::MotionPlanResponse bootstrap;
    if (!context->solve(bootstrap)) throw std::runtime_error("Initial native PRM query failed");
    auto ss = context->getOMPLSimpleSetup();
    ss->setup();
    auto prm = std::dynamic_pointer_cast<og::PRM>(ss->getPlanner());
    if (!prm) throw std::runtime_error("Planner is not native PRM");
    const auto begin = std::chrono::steady_clock::now();
    // This builds throughout the requested time, without stopping at the first start-goal path.
    prm->constructRoadmap(ob::timedPlannerTerminationCondition(build_seconds));
    const double build_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    const auto status = ss->solve(3.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) throw std::runtime_error("Built map, but verification query failed");
    ob::PlannerData data(ss->getSpaceInformation());
    prm->getPlannerData(data);
    // PlannerData exports have default edge weights; restore the actual native distance costs.
    data.computeEdgeWeights();
    ob::PlannerDataStorage storage;
    if (!storage.store(data, (folder + "/roadmap.graph").c_str())) throw std::runtime_error("Cannot save roadmap");
    auto space = context->getOMPLStateSpace();
    auto coordinates = [&](const ob::State* state) {
      auto robot = scene->getCurrentState();
      space->copyToRobotState(robot, state);
      std::vector<double> q;
      robot.copyJointGroupPositions("left_arm", q);
      for (auto& value : q) value *= 180.0 / M_PI;
      return q;
    };
    auto writeCoordinates = [&](std::ostream& stream, const ob::State* state) {
      const auto values = coordinates(state);
      stream << '[';
      for (std::size_t i = 0; i < values.size(); ++i) stream << (i ? "," : "") << values[i];
      stream << ']';
    };
    std::ofstream graph(folder + "/graph.json");
    graph << std::setprecision(17) << "{\"nodes\":[";
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
      graph << (i ? "," : "") << "{\"id\":" << i << ",\"joints_deg\":";
      writeCoordinates(graph, data.getVertex(i).getState());
      graph << '}';
    }
    graph << "],\"edges\":[";
    bool first_edge = true;
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
      std::vector<unsigned int> neighbors;
      data.getEdges(i, neighbors);
      for (const auto j : neighbors)
      {
        ob::Cost weight;
        data.getEdgeWeight(i, j, &weight);
        graph << (first_edge ? "" : ",") << "{\"from\":" << i << ",\"to\":" << j
              << ",\"cost\":" << weight.value() << '}';
        first_edge = false;
      }
    }
    const auto& path = ss->getSolutionPath();
    graph << "],\"verification_path_g_h\":[";
    double g = 0;
    const auto* goal = path.getState(path.getStateCount() - 1);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
      if (i) g += ss->getSpaceInformation()->distance(path.getState(i-1), path.getState(i));
      const double h = ss->getSpaceInformation()->distance(path.getState(i), goal);
      graph << (i ? "," : "") << "{\"index\":" << i << ",\"joints_deg\":";
      writeCoordinates(graph, path.getState(i));
      graph << ",\"g\":" << g << ",\"h\":" << h << ",\"f\":" << g+h << '}';
    }
    graph << "]}\n";
    std::ostringstream result;
    result << std::setprecision(17) << "{\"build_s\":" << build_s
           << ",\"verification_search_s\":" << ss->getLastPlanComputationTime()
           << ",\"vertices\":" << data.numVertices() << ",\"directed_edges\":" << data.numEdges()
           << ",\"state_space\":" << std::quoted(space->getName())
           << ",\"native_state_space\":true,\"weighted_edges_recomputed\":true,\"verification_joint_path_cost\":"
           << path.length() << '}';
    std::ofstream(folder + "/build_result.json") << result.str() << '\n';
    std::cout << result.str() << std::endl;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
