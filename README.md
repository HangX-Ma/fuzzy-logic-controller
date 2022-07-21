# :rocket:`ZJUER:` A fuzzy controller with mixed integral and integral saturation rejection module

Realize the basic function of fuzzy logic controller. User can select _Triangle_, _Rectangle_, _Trapezoid_, _Gaussian_ membership functions to do fuzzification. _Centroid (COG)_ method is used to defuzzification. To enhance the performance of this controller, mix the fuzzy controller with a parallel integrator and add a module to prevent integral saturation.

## Schedule
- [x] The main body of the program is completed
- [x] Bugs have been fixed
- [x] Add an example. Visualization uses`matplotlib-cpp` wrapper. [lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp) provides this support
- [x] Interface and logic optimization
- [x] Protection checker
- [x] `README.md` update
- [x] Add integral and integral saturation rejection module

## Example Usage
Compile the project once you download this repository.

```shell {.line-numbers}
git clone https://github.com/HangX-Ma/fuzzy-logic-controller.git
cd fuzzy-logic-controller
mkdir build && cd build
cmake ..
cmake --build .
```
After you run the executable file, the following result will show on terminal and you can find `example.png` in share folder.

```shell {.line-numbers}
------------ Information of fuzzy logic controller ------------
Universal discourse [err]: [-640.000000, 640.000000]
Universal discourse [err_dev]: [-640.000000, 640.000000]
Universal discourse [u]: [-180.000000, 180.000000]
Kp_e=0.400000, Ki_e=0.010000, Kd_e=0.050000, Kp_u=4.000000, K_sat=0.100000
```
![example](share/example.png)
## Fuzzy Logic Control Basic Concept
Fuzzy logic controller is composed of the following four elements:
1. *A rule-base* (a set of If-Then rules), which contains a fuzzy logic quantification of the expert’s linguistic description of how to achieve good control.
2. *An inference mechanism* (also called an “inference engine” or “fuzzy inference” module), which emulates the expert’s decision making in interpreting and applying knowledge about how best to control the plant.
3. *A fuzzification interface*, which converts controller inputs into information that the inference mechanism can easily use to activate and apply rules.
4. *A defuzzification interface*, which converts the conclusions of the inference mechanism into actual inputs for the process.
![fzc-architecture](share/fzc-architecture.png)

## Modified Fuzzy Logic Controller

- Add a parallel integrator to eliminate steady-state error.
- Compare the clamped output $u_s$ with the original output $u$. This feedback value will be sent to integrator input channel to prevent integral saturation.

![fzc-modified](share/fuzzy-controller-modified.png)


## LICENSE
Apache License Version 2.0

## REF
> _Fuzzy Control_, Kevin M. Passino, Stephen Yurkovich, Department of Electrical Engineering, The Ohio State University  
> [fuzzylite/fuzzylite](https://github.com/fuzzylite/fuzzylite)  
> [shuoyueqishi/fuzzy-controller](https://github.com/shuoyueqishi/fuzzy-controller)  

