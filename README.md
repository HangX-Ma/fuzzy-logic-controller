# A fuzzy logic controller

Realize the basic function of fuzzy logic controller. User can select _Triangle_, _Trapezoid_, _Gaussian_ membership functions to do **fuzzification**. The **inference** process will select the highest weight rule, if exist some rules that points to the same output discourse. _Centroid (COG)_ method is used to **defuzzification**.

You can use _**P-FC**_ controller that combines _proportional controller_ with _fuzzy controller_. This will enhance the system response time and accuracy when input _error_ or _error rate_ high.

> **NOTE:** Fuzzy control can make control more concise, but also make it slow. I use forward feedback in demo to increase the control speed!

<div align="center">
    <img src="assets/fzc_design.svg" alt="Fuzzy logic controller design, HangX-Ma" width=600 />
    <br>
    <font size="2" color="#999"><u>Fuzzy logic controller design, HangX-Ma</u></font>
    <br></br>
</div>

## Environment

- Python3.11, matplotlib-3.7.1
- Eigen-3.4.0
- matplotlib-cpp
- C++17, CMake 3.23

## Prerequisite

- Install `spdlog` development components.

    ```bash
    sudo apt-get install libspdlog-dev
    ```

- Install `Eigen3` to acquire linear algebra calculation ability.

    ```bash
    wget -O Eigen.zip https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
    unzip Eigen.zip # it has unzipped into the library called eigen-3.4.0
    cd eigen-3.4.0
    cmake -B build
    sudo cmake --build build --target install # install Eigen3
    ```

## Schedule

- [x] Rewrite the core logic and fix the essential fuzzy logic problems.
- [x] Uses [lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp) wrapper to do visualization.
- [x] Refactor code following [Google C++ open source project style guide](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/).

## Usage

Compile the project after you download this repository. You can find the output executable file in `build/bin`.

```shell
git clone https://github.com/HangX-Ma/fuzzy-logic-controller.git
cd fuzzy-logic-controller
cmake -B build -S .
cmake --build build
# run the demo!
./build/bin/fuzzy_demo
```

You can also include this repository and compile it as a shared or static library. `libfuzzy.a` or `libfuzzy.so` will be found in `build` folder.

This repository only provide three membership functions: _Triangle_, _Trapezoid_, _Gaussian_. You need to provide your configuration parameters by yourself.

<table>
<tr>
    <td><img src="assets/u_membership_triangle.png" alt="Membership function: Triangle, HangX-Ma" width=400 /></td>
    <td><img src="assets/u_membership_trapezoid.png" alt="Membership function: Trapezoid, HangX-Ma" width=400 /></td>
    <td><img src="assets/u_membership_gaussian.png" alt="Membership function: Gaussian, HangX-Ma" width=400 /></td>
</tr>
<tr>
    <td align="center"><font size="2" color="#999"><u>Membership function: Triangle, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>Membership function: Trapezoid, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>Membership function: Gaussian, HangX-Ma</u></font></td>
</tr>
</table>

You can annotate these definitions in `CMakeLists.txt` to switch off/on those relative functions.

```cmake
# CMakeLists.txt
add_definitions(
    "-DFC_USE_DOUBLE"
    "-DFC_USE_MATPLOTLIB"
)
```

## Demo

<div align="center">
    <img src="assets/fuzzy_control_surface.png" alt="Fuzzy control surface, HangX-Ma" width=400 />
    <br>
    <font size="2" color="#999"><u>Fuzzy control surface, HangX-Ma</u></font>
    <br></br>
</div>

```bash
[2024-01-16 22:18:41.579] [info] Fuzzy logic controller info:
[2024-01-16 22:18:41.579] [info] => discourse  e: [-60.000, 60.000], min-max[-3.00, 3.00]
[2024-01-16 22:18:41.579] [info] => discourse ec: [-30.000, 30.000], min-max[-3.00, 3.00]
[2024-01-16 22:18:41.579] [info] => discourse  u: [-15.000, 15.000], min-max[-3.00, 3.00]
[2024-01-16 22:18:41.579] [info] => error quantifying factor             [Ke]: 0.0500
[2024-01-16 22:18:41.579] [info] => derivative error quantifying factor [Kec]: 0.1000
[2024-01-16 22:18:41.579] [info] => output scaling factor                [Ku]: 5.0000
[2024-01-16 22:18:41.579] [info] => proportional controller              [Kp]: 2.0000
```

### Fuzzy Controller

<table>
<tr>
    <td><img src="assets/fc_demo_target_and_actual_constant.png" alt="FC Demo - Constant: Target and Actual, HangX-Ma" width=500 /></td>
    <td><img src="assets/fc_demo_err_and_derr_constant.png" alt="FC Demo - Constant: Error and Derivative Error, HangX-Ma" width=500 /></td>
</tr>
<tr>
    <td align="center"><font size="2" color="#999"><u>FC Demo - Constant: Target and Actual, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>FC Demo - Constant: Error and Derivative Error, HangX-Ma</u></font></td>
</tr>
</table>

<table>
<tr>
    <td><img src="assets/fc_demo_target_and_actual_sine.png" alt="FC Demo - Sine: Target and Actual, HangX-Ma" width=500 /></td>
    <td><img src="assets/fc_demo_err_and_derr_sine.png" alt="FC Demo - Sine: Error and Derivative Error, HangX-Ma" width=500 /></td>
</tr>
<tr>
    <td align="center"><font size="2" color="#999"><u>FC Demo - Sine: Target and Actual, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>FC Demo - Sine: Error and Derivative Error, HangX-Ma</u></font></td>
</tr>
</table>

### Proportional + Fuzzy Controller

<table>
<tr>
    <td><img src="assets/pfc_demo_target_and_actual_constant.png" alt="P-FC Demo - Constant: Target and Actual, HangX-Ma" width=500 /></td>
    <td><img src="assets/pfc_demo_err_and_derr_constant.png" alt="P-FC Demo - Constant: Error and Derivative Error, HangX-Ma" width=500 /></td>
</tr>
<tr>
    <td align="center"><font size="2" color="#999"><u>P-FC Demo - Constant: Target and Actual, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>P-FC Demo - Constant: Error and Derivative Error, HangX-Ma</u></font></td>
</tr>
</table>

<table>
<tr>
    <td><img src="assets/pfc_demo_target_and_actual_sine.png" alt="P-FC Demo - Sine: Target and Actual, HangX-Ma" width=500 /></td>
    <td><img src="assets/pfc_demo_err_and_derr_sine.png" alt="P-FC Demo - Sine: Error and Derivative Error, HangX-Ma" width=500 /></td>
</tr>
<tr>
    <td align="center"><font size="2" color="#999"><u>P-FC Demo - Sine: Target and Actual, HangX-Ma</u></font></td>
    <td align="center"><font size="2" color="#999"><u>P-FC Demo - Sine: Error and Derivative Error, HangX-Ma</u></font></td>
</tr>
</table>

## Fuzzy Logic Control Basic Concept

Fuzzy logic controller is composed of the following four elements:

1. A _**rule-base**_ (a set of If-Then rules), which contains a fuzzy logic quantification of the expert’s linguistic description of how to achieve good control.
2. An _**inference mechanism**_ (also called an “inference engine” or “fuzzy inference” module), which emulates the expert’s decision making in interpreting and applying knowledge about how best to control the plant.
3. A _**fuzzification interface**_, which converts controller inputs into information that the inference mechanism can easily use to activate and apply rules.
4. A _**defuzzification interface**_, which converts the conclusions of the inference mechanism into actual inputs for the process.

<div align="center">
    <img src="assets/fzc_architecture.png" alt="Fuzzy controller architecture, HangX-Ma" width=600 />
    <br>
    <font size="2" color="#999"><u>Fuzzy controller architecture, HangX-Ma</u></font>
</div>

## Reference

- [Fuzzy Control](share/FCbook.pdf), Kevin M. Passino, Stephen Yurkovich, Department of Electrical Engineering, The Ohio State University
- [fuzzylite/fuzzylite](https://github.com/fuzzylite/fuzzylite)
- [shuoyueqishi/fuzzy-controller](https://github.com/shuoyueqishi/fuzzy-controller)
- [{fmt} Formatting & Printing Library](https://hackingcpp.com/cpp/libs/fmt.html)

## License

MIT License
