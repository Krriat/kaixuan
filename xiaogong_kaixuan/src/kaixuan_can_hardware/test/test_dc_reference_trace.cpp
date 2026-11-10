#include "kaixuan_can_hardware/dc_reference_trace.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using Ring = kaixuan_can_hardware::DcReferenceTraceRing;
static unsigned checks = 0;
#define CHECK(condition) do {++checks; if (!(condition)) { \
  std::cerr << __LINE__ << ": " #condition "\n"; std::exit(1);}} while (false)

int main()
{
  auto ring = std::make_unique<Ring>();
  CHECK(ring->size() == 0 && !ring->frozen() && ring->overwritten() == 0);
  Ring::Row row;
  row.cycle = 1;
  row.previous_send_cycle = 0;
  row.previous_raw = {100, 90, 104, true};
  row.residual_ns = -20215;
  row.filtered_ns = -16853.740232;
  row.flags = Ring::ResidualAvailable | Ring::PreviousRawValid;
  row.control.fill(6);
  row.status.fill(0x231);
  CHECK(ring->push(row, false));
  CHECK(ring->chronological(0).previous_raw.raw_ns == 90);
  CHECK(ring->chronological(0).residual_ns == -20215);
  CHECK(ring->chronological(0).filtered_ns == row.filtered_ns);
  CHECK(ring->chronological(0).status[6] == 0x231);
  for (std::uint64_t cycle = 2; cycle <= Ring::kCapacity; ++cycle) {
    row.cycle = cycle;
    row.previous_send_cycle = cycle - 1;
    CHECK(ring->push(row, false));
  }
  CHECK(ring->size() == Ring::kCapacity && ring->overwritten() == 0);
  CHECK(ring->chronological(0).cycle == 1);
  for (std::uint64_t cycle = Ring::kCapacity + 1; cycle <= Ring::kCapacity * 2 + 17; ++cycle) {
    row.cycle = cycle;
    row.previous_send_cycle = cycle - 1;
    CHECK(ring->push(row, false));
  }
  CHECK(ring->overwritten() == Ring::kCapacity + 17);
  for (std::size_t i = 0; i < ring->size(); ++i) {
    const auto & saved = ring->chronological(i);
    CHECK(saved.cycle == Ring::kCapacity + 18 + i);
    CHECK(saved.previous_send_cycle + 1 == saved.cycle);
  }
  row.cycle = Ring::kCapacity * 2 + 18;
  row.flags |= Ring::FollowerFailed;
  row.failure = 7;
  CHECK(ring->push(row, true));
  CHECK(ring->frozen() && ring->first_failure_cycle() == row.cycle);
  CHECK(ring->chronological(ring->size() - 1).failure == 7);
  const auto total = ring->total(), overwritten = ring->overwritten();
  for (unsigned i = 0; i < 100; ++i) {++row.cycle; CHECK(!ring->push(row, false));}
  CHECK(ring->total() == total && ring->overwritten() == overwritten);
  CHECK(ring->chronological(ring->size() - 1).cycle == ring->first_failure_cycle());
  auto early = std::make_unique<Ring>();
  row.cycle = 1;
  CHECK(early->push(row, true));
  CHECK(early->size() == 1 && early->chronological(0).cycle == 1 && early->frozen());
  std::ostringstream output;
  early->write_csv(output, 1);
  CHECK(output.str().find("# first_failure_cycle,1\n") != std::string::npos);
  CHECK(output.str().find("3_RAW_bracket_plus_2_if_getter_called") != std::string::npos);
  std::istringstream csv(output.str());
  std::string line;
  std::vector<std::vector<std::string>> csv_rows;
  while (std::getline(csv, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream fields(line);
    std::vector<std::string> cells;
    std::string cell;
    while (std::getline(fields, cell, ',')) {cells.push_back(cell);}
    csv_rows.push_back(cells);
  }
  CHECK(csv_rows.size() == 2 && csv_rows[0].size() == 68 && csv_rows[1].size() == 68);
  CHECK(csv_rows[0][0] == "cycle" && csv_rows[1][0] == "1");
  CHECK(csv_rows[0][20] == "residual_ns" && csv_rows[1][20] == "-20215");
  CHECK(csv_rows[0][21] == "filtered_ns" && std::stod(csv_rows[1][21]) == row.filtered_ns);
  CHECK(csv_rows[0][14] == "previous_raw_mono_before_ns" && csv_rows[1][14] == "100");
  CHECK(csv_rows[0][64] == "axis6_error" && csv_rows[1][64] == "0");
  CHECK(csv_rows[0][66] == "previous_send_host_ns");
  std::cout << "PASS dc_reference_trace " << checks << " checks; row=" << sizeof(Ring::Row)
            << " ring=" << sizeof(Ring) << " bytes\n";
}
