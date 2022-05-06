# NSF-CPS-Challenge-2021
This is TEAM GREEN's implementation of the NSF CPS Challenge 2021 which tasks us to throw the sensor probe on a Polaris Ranger

Ranger_Detection.ipynb: The  code is the implementation to detect the Polaris ranger, precisely the trunk of the ranger using masking, morphological transformations including erosion followed by dilation. This is implemented using OpenCV and Python.
After successfully masking the trunk, we surround it using a rectangle and proceed to find the centroid of the rectangle as our throw target point.

<img width="317" alt="image" src="https://user-images.githubusercontent.com/29693118/167205272-d213a683-2f23-4be6-bdbc-64c3e2115991.png">
<img width="296" alt="image" src="https://user-images.githubusercontent.com/29693118/167205376-62b538f3-b982-4262-ac1a-faa8f9b114a5.png">
<img width="316" alt="image" src="https://user-images.githubusercontent.com/29693118/167205386-629238b7-cc13-4b98-a2dd-86d5275bf9e8.png">

