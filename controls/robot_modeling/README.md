## modeling-old

Contains all the old robot modeling done around the year 2015-2016. This corresponds to PR #322 and issue #98. Although not completely confirmed, there may be some mistakes as models in other papers have additional terms that are not accounted for, especially related to the BLDC motor.

## model-predict-nn

Uses a nural net to try and predict the ""state space" model

## plant_excitation

Compares state space model to the real data.

Additionally, it also finds the LQR coefficients for a body vel -> motor voltage controller on the robot.

## simulink_model

Compares our actual data with various different control strategies, specifically LQR and LQI.

It also tries various encoder feedback mechanisms to improve position estimate on the soccer computer to account for vision lag.

Finally, it compares the descrete and continous response of the state space models.

## State Space.docx

State space model with various different complexity levels depending onthe coordinate frames.