# WaveGuard: AXI-Based Multi-Master Interconnect for SoC Communication

## Overview

WaveGuard is a System-on-Chip (SoC) communication infrastructure project that implements a scalable AXI-based interconnect supporting multiple masters and slaves. The design enables efficient data transfer, arbitration, and address decoding between processing elements and peripherals.

The project focuses on developing a reliable and configurable interconnect architecture suitable for FPGA and ASIC-based embedded systems.

## Features

* AXI-compliant communication architecture
* Multi-master support (3 Masters)
* Multi-slave support (10 Slaves)
* Priority-based arbitration
* Address decoding and routing
* Modular Verilog RTL design
* Simulation and verification environment
* Scalable architecture for future expansion

## Project Structure

```text
rtl/
└── interconnect/
    ├── axi_interconnect.v
    ├── arbiter.v
    └── priority_encoder.v

tb/
├── tb_axi_interconnect_3x10.sv
└── axi_slave_mem.sv

runfile.f

docs/
└── WaveGuard_Architecture_Update.pdf
```

## Architecture

The WaveGuard interconnect connects multiple AXI masters to multiple AXI slaves through:

1. Address Decoder

   * Identifies the target slave based on address ranges.

2. Arbiter

   * Resolves simultaneous requests from multiple masters.

3. Priority Encoder

   * Selects the highest-priority request.

4. Routing Logic

   * Directs read/write transactions to the selected slave.

## Verification

The design is verified using SystemVerilog testbenches.

Verification includes:

* Read transactions
* Write transactions
* Multiple-master contention
* Arbitration validation
* Address decoding verification
* Slave selection verification

## Simulation

Compile and run using the provided file list:

```bash
vlog -f runfile.f
vsim tb_axi_interconnect_3x10
run -all
```

Generate waveforms for analysis:

```bash
add wave *
run -all
```

## Applications

* Embedded SoCs
* FPGA Prototyping
* Multi-core Communication Systems
* Custom ASIC Designs
* Research and Academic Projects

## Future Enhancements

* Round-robin arbitration
* QoS-aware scheduling
* AXI4 full protocol support
* Performance monitoring counters
* Configurable address maps
* UVM-based verification environment

## Author

Bhupathi Reddy

B.E. Electronics and Communication Engineering

WaveGuard SoC Communication Project
