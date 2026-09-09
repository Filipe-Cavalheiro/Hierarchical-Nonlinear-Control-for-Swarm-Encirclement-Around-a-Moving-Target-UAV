
## Running the animation
```bash
# Create a virtual environment (first time only)
python3 -m venv .venv

# Activate the virtual environment
# macOS/Linux
source .venv/bin/activate

# Windows (Command Prompt)
venv\Scripts\activate

# Windows (PowerShell)
venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt

# Run the presentation (opengl rendere might fail)
manim slides.py TitleCard Motivation Intro Objective FibonacciSphere ControllerDesign APF Radial_control Conclusions Results  --renderer=opengl

# Back up
manim slides.py TitleCard Motivation Intro Objective FibonacciSphere ControllerDesign APF Radial_control Conclusions Results  

# Present locally
manim-slides TitleCard Motivation Intro Objective FibonacciSphere ControllerDesign APF Radial_control Conclusions Results  --fullscreen

# Share presentation as a HTML file
manim-slides convert TitleCard Motivation Intro Objective FibonacciSphere ControllerDesign APF Radial_control Results Conclusions presentation.html --one-file --offline
```