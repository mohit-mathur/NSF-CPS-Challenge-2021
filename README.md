# NSF-CPS-Challenge-2021
This is TEAM GREEN's implementation of the NSF CPS Challenge 2021 which tasks us to throw the sensor probe on a Polaris Ranger


Phase1A.py: This file contains the implementation of Phase 1A of the NSF challenge 
Task: Throw the probe on the target and as close to the bullseye as possible.

<img width="968" alt="image" src="https://user-images.githubusercontent.com/29693118/167206286-f840c697-9f91-4408-8518-efb24c75ff2b.png">


https://user-images.githubusercontent.com/29693118/167207133-c3f3a336-49c9-4055-89ae-3aa9b867a48a.mp4




Ranger_Detection.ipynb: The  code is the implementation to detect the Polaris ranger, precisely the trunk of the ranger using masking, morphological transformations including erosion followed by dilation. This is implemented using OpenCV and Python.
After successfully masking the trunk, we surround it using a rectangle and proceed to find the centroid of the rectangle as our throw target point.

<img width="317" alt="image" src="https://user-images.githubusercontent.com/29693118/167205272-d213a683-2f23-4be6-bdbc-64c3e2115991.png">   <img width="296" alt="image" src="https://user-images.githubusercontent.com/29693118/167205376-62b538f3-b982-4262-ac1a-faa8f9b114a5.png">   <img width="316" alt="image" src="https://user-images.githubusercontent.com/29693118/167205386-629238b7-cc13-4b98-a2dd-86d5275bf9e8.png">




Phase1B.py: This file contains implementation of Phase 1B of the Challenge.
Task: Throw the sensory probe by performing agile maneuvers of the UAV on the trunk of the Polaris Ranger which is to be detected using an onboard camera.
Implementation: Object detection using algorithm presented in Ranger_Detection.ipynb followed by aligning the UAV to the centroid of the rectangle marking the trunk of the Polaris Ranger. Finally, we attempt the “Belly-Flop” of the UAV and adjust the parachute deployment time such that the sensory probe lands on the trunk.


https://user-images.githubusercontent.com/29693118/167207065-1b4f2007-6cc0-4266-9361-8d1e4696286d.mp4


