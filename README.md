# Attitude Control

This code base is using the Julia Language and [DrWatson](https://juliadynamics.github.io/DrWatson.jl/stable/)
to make a reproducible scientific project named

## How-to
At the root of the project, start a new Julia terminal (tested with Julia 1.6.3), and run the following to instantiate

```julia
using DrWatson
@quickactivate "Attitude Control" # Activate the DrWatson project

import Pkg
Pkg.update() # Make sure all dependencies is installed
```

Then, you can use your favorite notebook editor/viewer to test out and use the notebooks found under `notebooks/`