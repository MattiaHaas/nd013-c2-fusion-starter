# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
The first part was about the  Kalman Filter given Lidar measurements. For the measurement update I implemented the measurement model and for the prediction update I used a constant velocity model.

Then I implemented the track management which enabled me to track any car in any image by using the first sensor measurement as the initial guess. Then I also added the track score and state in order to be able to track the car over subsequent images.

The association step enabled the system to track multiple vehicles at the same time.

The camera fusion was the hardest step for my as I initially forgot to transform the predicted state to the camera sensor coordinates.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 

Looking at my results from ![Step 3](img/final_project_3.PNG "Lidar only tracking RMSE results") and ![Step 4](img/final_project_4.PNG "Lidar and camera tracking RMSE results")  I can see that the RMSE has decreased substantially by adding the camera information to the KF.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
I could see many False Positives in terms of measurement from both the Lidar as well as the camera. I suspect that this could be worse in less ideal conditions like for example rainy nights

### 4. Can you think of ways to improve your tracking results in the future?
One could definitely think about tuning the process and measurement covariances to adapt them to the sensors.
