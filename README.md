# Resource Constrained Scheduling Algorithms Library (`LpScheduler`)
We solve the following problem:

Given a Data Flow Graph (or a Directed Acyclic Graph)  `G(V,E)`  with *nodes* corresponding to *instructions*,  *edges* corresponding to *data dependencies*,  a *resource function* `r : V->N` associated with nodes and *resource constraints* on some *abstract resouce* and a *delay function* `d: V->N` associated with nodes coressponding to *clock cycles*. This library provides algorithms to generate an *asynchronous* schedule which meets the resource constraints.

There are several types of abstract resources (*discrete set*, *counting set* , *contiguous memory* etc..) the library provides algorithm to generate a *feasible schedule*. Additionally for the special case of *contigous memory resource* the algorithm even modifies the input data-flow to generate a feasible schedule.

The *feasible schedule* is then used as a *basic feasible solution* to the underlying *Integer Linear Program (ILP)* to optimize the throughput of the schedule. For the special case of *contiguous memory resource* the *ILP* is equivalent to the *2D Strip Packing Problem*.

## Throughput Optimization Illustration:
The class `Strip_Packing_Linear_Program` takes a set of bins `B={b_1, b_2 ... b_n}` and generates LP formulation for a standard 2D Strip Packing problem. The generated constraint matrix is passed to GLPK solver to generate a solution. The details on how these linear constraints are generated is presented in [[1]](https://intel-my.sharepoint.com/:w:/p/vamsi_k_kundeti/EfLet7oMOrJHii-iMj65recBywuUHw2NvMznGgSiNl5ovw?e=1sILej)


Following is an example `B = { (2,2) , (8,8), (7,7) }` with 3 bins and we want to pack into a strip of dimensions `15 x 15`.
```
Minimize                                                                           
 obj: + z_H                                                                        
                                                                                   
Subject To                                                                         
 ROW~1: - 30 t_U_1_2 - x_2 + x_1 <= -2                                             
 ROW~2: - 30 t_L_1_2 - x_1 + x_2 <= -8                                             
 ROW~3: - 30 t_B_1_2 - y_2 + y_1 <= -2                                             
 ROW~4: - 30 t_R_1_2 - y_1 + y_2 <= -8                                             
 ROW~5: + t_R_1_2 + t_B_1_2 + t_L_1_2 + t_U_1_2 <= 3                               
 ROW~6: - 30 t_U_1_3 - x_3 + x_1 <= -2                                             
 ROW~7: - 30 t_L_1_3 - x_1 + x_3 <= -7                                             
 ROW~8: - 30 t_B_1_3 - y_3 + y_1 <= -2                                             
 ROW~9: - 30 t_R_1_3 - y_1 + y_3 <= -7                                             
 ROW~10: + t_R_1_3 + t_B_1_3 + t_L_1_3 + t_U_1_3 <= 3                              
 ROW~11: - 30 t_U_2_3 - x_3 + x_2 <= -8                                            
 ROW~12: - 30 t_L_2_3 - x_2 + x_3 <= -7                                            
 ROW~13: - 30 t_B_2_3 - y_3 + y_2 <= -8                                            
 ROW~14: - 30 t_R_2_3 - y_2 + y_3 <= -7                                            
 ROW~15: + t_R_2_3 + t_B_2_3 + t_L_2_3 + t_U_2_3 <= 3                              
 ROW~16: + x_1 <= 13                                                               
 ROW~17: + x_2 <= 7                                                                
 ROW~18: + x_3 <= 8                                                                
 ROW~19: - z_H + y_1 <= -2                                                         
 ROW~20: - z_H + y_2 <= -8                                                         
 ROW~21: - z_H + y_3 <= -7                                                         
 ROW~22: + z_H <= 15                                                               
The following is the solution generated:

Constructing initial basis...                                                      
Size of triangular part is 18                                                      
Solving LP relaxation...                                                           
GLPK Simplex Optimizer, v4.65                                                      
18 rows, 19 columns, 54 non-zeros                                                  
      0: obj =   8.000000000e+00 inf =   1.275e+01 (12)                            
     12: obj =   8.000000000e+00 inf =   0.000e+00 (0)                             
OPTIMAL LP SOLUTION FOUND                                                          
Integer optimization begins...                                                     
Long-step dual simplex will be used                                                
+    12: mip =     not found yet >=              -inf        (1; 0)                
+    21: >>>>>   1.500000000e+01 >=   8.000000000e+00  46.7% (6; 0)                
+    36: >>>>>   1.000000000e+01 >=   8.000000000e+00  20.0% (9; 6)                
+    43: >>>>>   9.000000000e+00 >=   8.000000000e+00  11.1% (5; 15)               
+    43: mip =   9.000000000e+00 >=     tree is empty   0.0% (0; 29)               
INTEGER OPTIMAL SOLUTION FOUND                                                     
Obj = 9                                                                            
x_1 = 8.000000                                                                     
y_1 = 7.000000                                                                     
x_2 = 0.000000                                                                     
y_2 = 0.000000                                                                     
x_3 = 8.000000                                                                     
y_3 = 0.000000       
```
![title](https://user-images.githubusercontent.com/55415836/104082178-e0295600-51e8-11eb-96ca-92ee160e65d1.png)


## Feasible Solution to Optimal solution
Following is the input data-flow where *width* corresponds to the *resource utility* ,  *height* corresponds to *delay* and *edges* corresponding to *data dependencies*.

![input](https://user-images.githubusercontent.com/55415836/189461009-d30e9038-bf32-4a3b-8758-69260fb9f6dc.png)

Following is the *feasible schedule* with *make-span* of **57** and the next picture shows the optimized solution with *make-span* of **47**.

![feasible-solution](https://user-images.githubusercontent.com/55415836/189461012-00a024e1-c15b-4f60-941c-1952dc1a08e0.png)


![optimal-solution](https://user-images.githubusercontent.com/55415836/189461015-4581b4f3-d422-44cb-95f8-f6be5e57bf1c.png)

## Building Unit Tests

```
git clone ....
cd LpScheduler
mkdir build
cmake -DBUILD_UNIT_TESTS=True ../src/
make -j10
```
