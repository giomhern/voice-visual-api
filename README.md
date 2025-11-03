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
│   ├── app.py
│   ├── audio.py
│   ├── gemini.py
│   └── robot_control.py
├── misc
│   └── central_control_node.py
├── README.md
├── requirements.txt
└── tests
    └── test.wav
```

## Getting Started 

All of these steps should be performed on the Stretch robot.

1. Clone this repository. 

```
git clone
```

2. Create a virtual environment and activate it:
   
```
python -m venv myenv
source myenv/bin/activate
```

3. Install the required dependencies.

```
pip install -r requirements.txt
```

4. Navigate to the `app` directory.

```
cd app
```

5. Start the Flask API server:

```
python app.py
```

## Commands 

There are numerous commands you can run in a separate terminal window to ensure your robot can be accessed via API. 


### Health/Diagnositics

```
# health & diag
curl -s http://localhost:5000/health
curl -s http://localhost:5000/robot/diagnostics
```

For quick e-start verification:

```
# start (release run-stop/estop first)
curl -s -X POST http://localhost:5000/robot/start
```


### Movement

```
# move 0.2 m forward
curl -s -X POST http://localhost:5000/robot/move -H "Content-Type: application/json" -d '{"distance_m":0.2}'
```


### Speech to Modify Settings

```
arecord -f cd -d 4 -t wav test.wav # record 4 second voice snippet
# Say "start the robot"
curl -X POST http://localhost:5000/voice/command \
  -F "audio=@test.wav" \
  -F "mime_type=audio/wav"
```

```
# use only textual interface
curl -X POST http://localhost:5000/voice/command \
  -H "Content-Type: application/json" \
  -d '{"text":"mute"}'
```