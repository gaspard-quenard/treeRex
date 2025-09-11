# TreeRex


## Building

TreeRex use both [pandaPigrounder](https://github.com/panda-planner-dev/pandaPIgrounder) and [pandaPiParser](https://github.com/panda-planner-dev/pandaPIparser) which require the following dependancies:
- `g++` (C++17 support)
- `make`
- `flex`  (version 2.6 or higher)
- `bison` (versions 3.5.1 and 3.7.2 are known to work)
- `zip`
- `gengetopt` (tested with version 2.23)


You can build TreeRex like this:

```
make
```

The SAT solver to link SibylSat with can be set with the `IPASIRSOLVER` variable in the makefile. Valid values are `cadical`, `cryptominisat`, `glucose4`, `lingeling`, and `riss`.

Note that the Makefile in the base directory is only supposed to be used for building SibylSat [as an IPASIR application](https://github.com/biotomas/ipasir).


## Benchmarks

SibylSat includes benchmark sets from the totally-ordered HTN [IPC 2023](https://github.com/ipc2023-htn/ipc2023-domains). These benchmarks are included as submodules in the `Benchmarks` folder. To obtain the benchmarks, you will need to initialize the submodules as follows:

```bash
git submodule update --init --recursive
```

## Usage

TreeRex uses the HDDL file format.

Execute the planner executable like this:
```bash
./treerex path/to/domain.hddl path/to/problem.hddl [options]
```