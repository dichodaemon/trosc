# A road lane with reference coordinate frame and timestamp
Header header
float64 width # road width [m]
float64 ch    # road curvature
float64 cl    # variation of road curvature
int8 car_position 
float64 y0
float64 relative_yaw
# the road equation is y=ch*x^2/2 + cl*x^3/6 + lambda* width
# with lambda = 0 or 1

