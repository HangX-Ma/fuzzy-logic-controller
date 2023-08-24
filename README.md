# A fuzzy logic controller

Realize the basic function of fuzzy logic controller. User can select _Triangle_, _Trapezoid_, _Gaussian_ membership functions to do fuzzification. _Centroid (COG)_ method is used to defuzzification. To enhance the performance of this controller, mix the fuzzy controller with a parallel integrator and add a module to prevent integral saturation.

## Environment

- Python3.11, matplotlib-3.7.1
- Eigen-3.4.0
- matplotlib-cpp
- C++17, CMake 3.23

## Schedule

- [ ] Uses [lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp) wrapper to do visualization .
- [ ] Refactor code following [Google C++ open source project style guide](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/).

## Example Usage

Compile the project after you download this repository.

```shell
git clone https://github.com/HangX-Ma/fuzzy-logic-controller.git
cd fuzzy-logic-controller
cmake -B build -S .
cmake --build build
```

<div align="center">
    <img src="assets/fuzzy_control_surface.png" alt="Fuzzy control surface, HangX-Ma" width=400 />
    <br>
    <font size="2" color="#999"><u>Fuzzy control surface, HangX-Ma</u></font>
</div>

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

## License

MIT License
