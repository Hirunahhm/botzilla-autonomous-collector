

# Import the Cube-Dataset-version-01 in to Your notebook 
[Link](https://app.roboflow.com/botzillaiesl-robo-games/cube-detection-dataset/)

    !pip install roboflow

    from roboflow import Roboflow
    rf = Roboflow(api_key="###########")
    project = rf.workspace("botzillaiesl-robo-games").project("Cube-Detection-Dataset")
    dataset = project.version(1).download("yolov8")

