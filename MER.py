import math
import numpy as np
import matplotlib.pyplot as plt
##########
import sys

def getNumberOrbits():
    try:
        return float(sys.argv[1])
    except ValueError:
        print("Invalid Number of orbits. Setting to default value of 1")
        return 1

def getOrbitSpeed():
    try:
        return float(sys.argv[2]) / 100
    except ValueError:
        print("Invalid Orbit Speed. Setting to default value of 100")
        return 100.0


def getVelocityAngle():
    #returnValue = 0.0
    try:
        return float(sys.argv[3])
    except ValueError:
        print("Invalid Velocity Angle. Setting to default value of 0")
        return 0.0

def getFrames():
    try:
        return int(sys.argv[4])
    except ValueError:
        print("Invalid value for frames. Setting to default value of 20")
        returnValue = 20

    return returnValue

percentorbit = getNumberOrbits()
percentorbitalspeed = getOrbitSpeed()
velocityAngle = getVelocityAngle()
frames = getFrames()

print(" ")
print("        **NEW COMMAND**")
print("    Orbital Speed %:          ", percentorbitalspeed*100)
print("    Starting Velocity Angle:  ", velocityAngle)
print("    # of points to plot:      ", frames*percentorbit)
print(" ")

############

print("computing...")

# TRIGONOMETRY SETUP

def cos(x):
    return math.cos(x * math.pi / 180)

def sin(x):
    return math.sin(x * math.pi / 180)

def mag(x):
    return np.linalg.norm(x)

# CONSTANTS
G = 6.67E-11  # m/s^2
earthMass = 5.972E24 # kg
marsMass = 0*earthMass#6.39E24 # kg
moonMass = 7.348E22  # kg
rocketMass = 50000 # kg
earthRadius = 6.371E6  # meters, radius of earth
marsRadius = 3.39E6
moonRadius = 3E5  # meters, radius of moon

# INPUTS
moonangle = 90  # deg, moon beginning position
moonvelocityangle = moonangle - 90 # deg
marsangle = 0  # deg
rocketangle = moonangle # deg
moondis = 4.066E8  # meters
marsdis = 8.066E8 # meters
rocketmiledis = 4000 # miles
rocketdis = moonRadius + rocketmiledis * 5280 * .3048 # FROM MOON SURFACE! meters
startTime = 0  # seconds
timeDelta = 60 # seconds
##percentorbit = float(input("How many orbits do you want? ")) # number of orbits moon would undergo under normal conditions
endTime = 2 * math.pi * moondis / ((G * earthMass / moondis) ** (1 / 2.0)) * percentorbit # seconds

# INITIAL CONDITIONS
positionEarth = np.array([0, 0, 0])  # [x,y,z]
positionMoon = np.array([moondis * cos(moonangle), moondis * sin(moonangle), 0])  # [x,y,z]
positionMars = np.array([marsdis * cos(marsangle), marsdis * sin(marsangle), 0])
positionRocket = np.array([moondis * cos(moonangle) + rocketdis * cos(rocketangle),moondis * sin(moonangle) + rocketdis * sin(rocketangle),0])
#positionRocket = 0 #not coded yet
vectorFromEarthToMoon = (positionMoon - positionEarth) / mag((positionMoon - positionEarth))  # unit vector from earth to moon
vectorFromMarsToMoon = (positionMoon - positionMars) / mag((positionMoon - positionMars))  # unit vector from mars to moon

# COMMAND INPUTS
##percentorbitalspeed = float(input("Percent of Orbital Speed: ")) / 100
##velocityAngle = float(input("Starting Velocity Angle: "))  # degrees, angle of the velocity with respect to x axis

V_m = (G * earthMass / moondis) ** (1 / 2.0) # m/s, orbital speed of moon
V_r = (G * moonMass / rocketdis) ** (1 / 2.0) * percentorbitalspeed # m/s, orbital speed osf rocket around moon
startingMoonVelocity = np.array([V_m * cos(moonvelocityangle), V_m * sin(moonvelocityangle), 0])  # vector velocity of moon at start
startingRocketVelocity = np.array([startingMoonVelocity[0] + V_r * cos(velocityAngle),startingMoonVelocity[1] + V_r * sin(velocityAngle),startingMoonVelocity[2] + 0])

# LOOP SECTION
currentTime = startTime  # seconds, tracks time

##frames = int(input("Enter the number of points to plot: "))
resolution = 2 * math.pi * moondis / ((G * earthMass / moondis) ** (1 / 2.0)) / frames

#CREATE EDGE BOUNDARIES FOR GRAPH
graphx = np.array([-1.5*moondis,-1.5*moondis,1.5*moondis,1.5*moondis])
graphy = np.array([-1.5*moondis,1.5*moondis,-1.5*moondis,1.5*moondis])

plt.ion() #flag plt as interactive
plt.figure(figsize=(10,10))

plt.plot([0], [0], color='blue', linestyle='dashed', marker='o', markerfacecolor='blue', markersize=10) # plots earth
#plt.plot([marsdis*cos(marsangle)], [marsdis*sin(marsangle)], 'ro') #Mars plotted
#plt.plot(graphx,graphy,'k^') #boundary points plotted
plt.plot(positionMoon[0], positionMoon[1], 'ko') #moon start point plotted
plt.plot(positionRocket[0], positionRocket[1], 'go') #moon start point plotted

#print ("endTime: ", endTime)
#print ("resolution: ", resolution)

outerLoopIterations = 0
totalInnerIterations = 0
crashmoon = 0
crashrocket = 0


#print("Beginning calculations")
while currentTime < endTime and (crashmoon < 1 or crashrocket < 1):
    z = 0  # reporting
    outerLoopIterations += 1
    ##print("Start Loop: ", outerLoopIterations)
    while z < resolution:
        totalInnerIterations += 1

        #positions and updating
        positionMoon = startingMoonVelocity * timeDelta + positionMoon # updates moon position
        positionRocket = startingRocketVelocity * timeDelta + positionRocket # updates rocket position

        #moon vectors
        rme = (positionEarth - positionMoon) / mag((positionEarth - positionMoon))  # unit vector from moon to earth
        rmm = (positionMars - positionMoon) / mag((positionMars - positionMoon)) # unit vector from moon to mars
        rmearth = mag((positionEarth - positionMoon)) # distance of moon to earth
        rmmars = mag((positionMars - positionMoon)) # distance of moon to mars
        #rocket vectors
        rre = (positionEarth - positionRocket) / mag((positionEarth - positionRocket)) # unit vector from rocket to earth
        rrm = (positionMoon - positionRocket) / mag((positionMoon - positionRocket)) # unit vector from rocket to moon
        rrmars = (positionMars - positionRocket) / mag((positionMars - positionRocket)) # unit vector from rocket to mars
        rrmagearth = mag((positionEarth - positionRocket)) # distance between rocket and earth
        rrmagmoon = mag((positionMoon - positionRocket)) # distance between rocket and moon
        rrmagmars = mag((positionMars - positionRocket)) # distance between rocket and mars

        #gravitational accelerations
        gearth = (G * earthMass / (mag(positionMoon - positionEarth)) ** 2) * rme # acceleration due to gravity on the moon due to earth
        gmars = (G * marsMass / (mag(positionMoon - positionMars)) ** 2) * rmm # acceleration due to gravity on the moon due to mars
        grearth = (G * earthMass / (mag(positionRocket - positionEarth)) ** 2) * rre # acceleration due to gravity on the moon due to
        grmoon = (G * moonMass / (mag(positionRocket - positionMoon)) ** 2) * rrm # acceleration due to gravity on the moon due to
        grmars = (G * marsMass / (mag(positionRocket - positionMars)) ** 2) * rrmars # acceleration due to gravity on the moon due to

        if rmearth < (earthRadius+moonRadius):
            startingMoonVelocity = 0
            crashmoon = 1
            startingRocketVelocity = (grearth + grmoon + grmars) * timeDelta + startingRocketVelocity
        elif rmmars < (marsRadius+moonRadius):
            startingMoonVelocity = 0
            crashmoon = 1
            startingRocketVelocity = (grearth + grmoon + grmars) * timeDelta + startingRocketVelocity
        else:
            startingMoonVelocity = (gearth + gmars) * timeDelta + startingMoonVelocity
            startingRocketVelocity = (grearth + grmoon + grmars) * timeDelta + startingRocketVelocity
        currentTime = currentTime + timeDelta
        z += timeDelta
    plt.plot(positionMoon[0], positionMoon[1], color='black', linestyle='dashed', marker='o', markerfacecolor='black', markersize=6) #plots current point of moon location
    plt.plot(positionRocket[0], positionRocket[1], color='green', linestyle='dashed', marker='o', markerfacecolor='green', markersize=3)#plots current point of rocket location
    plt.show()
    plt.pause(.0001)
    ##print("Finish Loop: ", outerLoopIterations)

# PRINT SECTION
# print ("Done computing. Please close graph to resume command window")
print (" ")

if crashmoon == 1:
    print("    moon has crashed.")
else:
	print("    moon has NOT crashed.")

if crashrocket == 1:
    print("    rocket has crashed.")
else:
	print("    rocket has NOT crashed.")

print (" ")

#plt.axis([-moondis * 1.25, moondis * 1.25, -moondis * 1.25, moondis * 1.25])
plt.show()

graphcommand = input("Would you like to close graph window (Y/N)? ")
if "graphcommand" == "Y" or "y" or "Yes" or "yes":
	plt.pause(1)
else:
	plt.pause(500) # Did this because the program will immediately exit since plt is no longer blocking the thread from continuing execution. sloppy but works

print(" ")

print(" GOT season 7 comes out tonight!! ")
