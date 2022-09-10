# Resource Constrained Scheduling Algorithms Library (`LpScheduler`)
We solve the following problem:

Given a Data Flow Graph (or a Directed Acyclic Graph)  `G(V,E)`  with *nodes* corresponding to *instructions*,  *edges* corresponding to *data dependencies* and a *resource function* `r : V->N` associated with nodes and *resource constraints* on some *abstract resouce*. This library provides algorithms to generate an *asynchronous* schedule which meets the resource constraints.

There are several types of abstract resources (*discrete set*, *counting set* , *contiguous memory* etc..) the library provides algorithm to generate a *feasible schedule*. Additionally for the special case of *contigous memory resource* the algorithm even modifies the input data-flow to generate a feasible schedule.

The *feasible schedule* is then used as a *basic feasible solution* to the underlying *Integer Linear Program (ILP)* to optimize the throughput of the schedule. For the special case of *contiguous memory resource* the *ILP* is equivalent to the *2D Strip Packing Problem*.

## Building Unit Tests

```
git clone ....
cd LpScheduler
mkdir build
cmake -DBUILD_UNIT_TESTS=True ../src/
make -j10
```
