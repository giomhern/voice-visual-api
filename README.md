# Voice vs Visual Interfaces API

This repository contains the source code for the Flask API built to control the settings of Hello Robot's Stretch. It allows us to have a REST API interface to connect with our Kotlin mobile application. 

## Research Abstract 

In our every-day interaction with digital mediums of all sorts (i.e. from robots to user
interfaces), personalization has become a critical step for establishing trust and usability.
In consumer robots, however, the process of personalization often requires the user to
become detached from the experience and switch to external devices such as
smartphones or web apps. Research in the field of human-computer interaction has
shown that voice interfaces can offer natural, intuitive ways to configure technology while
visual interfaces provide transparency and efficient feedback across complex tasks.
However, at large, we do not yet understand how the combination (and isolation) of each
of these modalities compare as it pertains to robot personalization and interaction. This
gap is exactly what this project tackles to better understand how voice-only, visual-only,
and hybrid interfaces influence our perceptions of control and primarily, the development
of mental models of tasks.

## File Structure

The file structure is shown below. `control.py` provides the heart of the API and `run.py` segments the API interface. All other files are included for project setup and completeness. 

```bash
├── app
│   ├── control.py
│   └── run.py
├── misc
│   └── central_control_node.py
├── README.md
└── requirements.txt
```

## Getting Started 

1. Clone this repository. 

```
git clone
```

2. Make sure all dependencies are installed. This should be configured to your virtual environment.

```
pip install -r requirements.txt
```

3. Navigate to the `app` directory.

```
cd app
```

4. Run on Hello Robot Stretch.

```
python3 -m venv .venv
source .venv/bin/activate
pip install flask flask-cors pyttsx3

# If amixer is missing (ALSA):
sudo apt-get update && sudo apt-get install -y alsa-utils

# Run
python app.py
```