# The python interface to the BeexyBox SDK

This contains a python module that loads the libbeexybox shared object. After
that the library is loaded all functions are translated to functions callable
by python. This yields a non-pythonic interface to a beexybox.
The library hence contains a number of classes that provide a better pythonic
interface where the BeexyBox is a type and events are handled via callable
classes.

## To Do
After there is an official package to the beexybox
