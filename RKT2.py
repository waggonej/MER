import math
import numpy as np
import matplotlib.pyplot as plt
import sys


    ## USER DEFINED INPUTS -- call cmd: python RKT2.py A B C
# A. Defines where on Earth the Rocket starts at
def getLaunchPadAngle():
    try:
        return float(sys.argv[1])
    except ValueError:
        print("Invalid Velocity Angle. Setting to default value of 0")
        return 0.0

# B. Defines the angle, relative to the ground, with which the rocket is initially pointing
def getVelocityAngle():
    try:
        return float(sys.argv[2])
    except ValueError:
        print("Invalid Velocity Angle. Setting to default value of 0")
        return 0.0

# C. Defines the initial speed with which the rocket begins
def launchspeed():
    try:
        return float(sys.argv[3])
    except ValueError:
        print("Invalid launch speed. Setting to default value of 0")
        return 0

LaunchPadLocationAngle = getLaunchPadAngle()
VelocityAngle = getVelocityAngle()
VelocityAngle += (LaunchPadLocationAngle - 90) #updates angle to make it relative to earth at all points
RocketSpeed = launchspeed()

print(" ")
print("        **NEW COMMAND**")
print("    Starting at an angle of:      ", LaunchPadLocationAngle)
print("    launch angle Rel to surface:  ", VelocityAngle)
print("    Initial Speed:                ", RocketSpeed)
print(" ")

print("computing...")

    ## TRIGONOMETRY SETUP
def cos(x):
    return math.cos(x * math.pi / 180) #Defines cos(x)

def sin(x):
    return math.sin(x * math.pi / 180) #Defines sin(x)

def mag(x):
    return np.linalg.norm(x) #Defines mag([x,y,z]) for magnitude

    ## CONSTANTS
# Universal Constants
G = 6.67E-11  # m/s^2
# Masses
earthMass = 5.972E24 # kg
rocketMass = 50000 # kg
# Radii
earthRadius = 6.371E6  # meters, radius of earth

    ## NON USER DEFINED INPUTS
# Angles

# Separation Distances
rocketdis = earthRadius + 1 # meters, launch pad relative to center of earth
boundary = earthRadius
# Time Tracking
startTime = 0  # seconds, beginning of launch
timeDelta = 1 # seconds, change in time per simulation loop
endTimeDays = 1
endTime = 100 # seconds, time before simulation close

    ## INITIAL CONDITIONS
# Position Vectors
positionEarth = np.array([0, 0, 0])  # [x,y,z]
positionRocket = np.array([rocketdis * cos(LaunchPadLocationAngle),rocketdis * sin(LaunchPadLocationAngle),0])
# Velocity Scalars
Vr = RocketSpeed # m/s, initial velocity of rocket
# Velocity Vectors
startingRocketVelocity = np.array([Vr * cos(VelocityAngle),Vr * sin(VelocityAngle),0])
# Create Edge Boundaries for Graph
graphx = np.array([-1.5*boundary,-1.5*boundary,1.5*boundary,1.5*boundary])
graphy = np.array([-1.5*boundary,1.5*boundary,-1.5*boundary,1.5*boundary])

    ## PLOTTING
plt.ion() # flag plt as interactive
plt.figure(figsize=(10,10))
# plots earth's curve
surfaceangle = 85 # initializes angle for circle graphing of earth
while surfaceangle < 95:
    plt.plot([earthRadius*cos(surfaceangle)], [earthRadius*sin(surfaceangle)], color='blue', linestyle='dashed', marker='o', markerfacecolor='blue', markersize=3) # plots earth
    surfaceangle += .5
# Starting Locations
plt.plot(positionRocket[0], positionRocket[1], 'go') # Rocket Starting Location
# Plot boundary points
plt.plot(graphx,graphy,'k^')

    ## LOOP SECTION of Plotting
currentTime = startTime  # seconds, initializes time tracking variable
timeframe = 1 # divides time and declares number of points to plot
resolution = timeframe # Amount of time between plotted points
# Counters
outerLoopIterations = 0
totalInnerIterations = 0
crashrocket = 0 # Checks for crash condition of Rocket
z = 0 # Resolution tracker

    ##THE LOOP
while currentTime < endTime and crashrocket < 1:
    outerLoopIterations += 1
    print("Start Loop: ", outerLoopIterations)
    while z < resolution:
        totalInnerIterations += 1
        #positions and updating
        positionRocket += startingRocketVelocity * timeDelta # updates rocket position
        #rocket vectors
        rre = (positionEarth - positionRocket) / mag((positionEarth - positionRocket)) # unit vector from rocket to earth
        rrmagearth = mag((positionEarth - positionRocket)) # distance between rocket and earth
        #gravitational accelerations
        grearth = (G * earthMass / (mag(positionRocket - positionEarth)) ** 2) * rre # acceleration due to gravity on the moon due to

        # Updates Velocity after checking for crash conditions
        if rrmagearth < (earthRadius):
            crashrocket = 1
            startingRocketVelocity = 0
        else:
            startingRocketVelocity += (grearth) * timeDelta
        # Updates time
        currentTime += timeDelta
        z += timeDelta
    plt.plot(positionRocket[0], positionRocket[1], color='green', linestyle='dashed', marker='o', markerfacecolor='green', markersize=3)#plots current point of rocket location
    plt.show()
    plt.pause(.0001)
    print("Finish Loop: ", outerLoopIterations)
#END

    ## PRINT SECTION
print (" ")

if crashrocket == 1:
    print("    rocket has crashed.")
else:
	print("    rocket has NOT crashed.")

print (" ")

graphcommand = input("Would you like to close graph window (Y/N)? ")
if "graphcommand" == "Y" or "y" or "Yes" or "yes":
	plt.pause(1)
else:
	plt.pause(500) # Did this because the program will immediately exit since plt is no longer blocking the thread from continuing execution. sloppy but works

print(" ")
