To build shared lib (on Ubuntu 18)

> sudo apt install libgsl23 libgslcblas0
> make PYBIND_INCLUDES="$(python3 -m pybind11 --includes)"

To run

copy libgsl.so.23 and libgslcblas.so.0 from Ubuntu 18 machine to root of binary 
export LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH
(import localization from python)