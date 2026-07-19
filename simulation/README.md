# Installation

1. Install [Python 3.12](https://www.python.org/ftp/python/3.12.0/)
    and make sure to add it to PATH. Then restart your IDE
2. Run these in the project folder to create a new virtual environment:

    `py -3.12 -m venv mujoco312`

    `mujoco312\Scripts\activate`

    If Powershell gives you trouble, you can run this:
    \
        `(Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned)`
3. Install the dependencies via the `requirements.txt`

    `pip install -r requirements.txt`

    (this installs the `nevergrad`, `mujoco`, and `keyboard` libraries, along with their dependencies at time of writing)

4. Run `runner.py` to play around with the model or use `gait_finder.py` to explore optimizations.
