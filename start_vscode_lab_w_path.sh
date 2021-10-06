#!/bin/bash
LD_LIBRARY_PATH="$(find -name "*.so" | sed 's:[^/]*$::' | sed 's/\./\.\./g' | tr '\n' ':')" code
