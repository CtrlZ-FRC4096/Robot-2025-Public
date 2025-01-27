from wpimath.geometry import Rotation2d, Rotation3d, Translation2d, Translation3d, Pose2d, Pose3d, Transform2d
from field_const import FieldConstants
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

## Plot and label the field

# Read the image
image_path = '2025 REEFSCAPE Transparent Background.png'
image = mpimg.imread(image_path)

# Create a figure and axis
fig, ax = plt.subplots(figsize=(20, 10))
ax.imshow(image, extent=[0, FieldConstants.fieldLength, 0, FieldConstants.fieldWidth])
# Set the aspect of the plot to be equal
ax.set_xlim(0, FieldConstants.fieldLength)
ax.set_ylim(0, FieldConstants.fieldWidth)
ax.set_aspect('equal')

## Add field elements to the plot
#Staging Positions
plt.scatter(FieldConstants.StagingPositions.leftIceCream.X(), FieldConstants.StagingPositions.leftIceCream.Y(), c='black')
plt.scatter(FieldConstants.StagingPositions.rightIceCream.X(), FieldConstants.StagingPositions.rightIceCream.Y(), c='black')
plt.scatter(FieldConstants.StagingPositions.middleIceCream.X(), FieldConstants.StagingPositions.middleIceCream.Y(), c='black')

red_leftIceCream = FieldConstants.flip_Translation2d(FieldConstants.StagingPositions.leftIceCream)
red_rightIceCream = FieldConstants.flip_Translation2d(FieldConstants.StagingPositions.rightIceCream)
red_middleIceCream = FieldConstants.flip_Translation2d(FieldConstants.StagingPositions.middleIceCream)

plt.scatter(red_leftIceCream.X(), red_leftIceCream.Y(), c='black')
plt.scatter(red_rightIceCream.X(), red_rightIceCream.Y(), c='black')
plt.scatter(red_middleIceCream.X(), red_middleIceCream.Y(), c='black')

#Processor
plt.scatter(FieldConstants.Processor.centerFace.X(), FieldConstants.Processor.centerFace.Y(), c='black')

red_centerFace = FieldConstants.flip_Translation2d(FieldConstants.Processor.centerFace)
plt.scatter(red_centerFace.X(), red_centerFace.Y(), c='black')

#Barge
plt.scatter(FieldConstants.Barge.farCage.X(), FieldConstants.Barge.farCage.Y(), c='black')
plt.scatter(FieldConstants.Barge.middleCage.X(), FieldConstants.Barge.middleCage.Y(), c='black')
plt.scatter(FieldConstants.Barge.closeCage.X(), FieldConstants.Barge.closeCage.Y(), c='black')

red_farCage = FieldConstants.flip_Translation2d(FieldConstants.Barge.farCage)
red_middleCage = FieldConstants.flip_Translation2d(FieldConstants.Barge.middleCage)
red_closeCage = FieldConstants.flip_Translation2d(FieldConstants.Barge.closeCage)

plt.scatter(red_farCage.X(), red_farCage.Y(), c='black')
plt.scatter(red_middleCage.X(), red_middleCage.Y(), c='black')
plt.scatter(red_closeCage.X(), red_closeCage.Y(), c='black')

#Reef

reef_colors = ['red', 'green', 'blue', 'yellow', 'purple', 'orange']

for i, reef_center in enumerate(FieldConstants.Reef.centerFaces):
    plt.scatter(reef_center.X(), reef_center.Y(), c=reef_colors[i])
    
    red_reef_center = FieldConstants.flip_Translation2d(reef_center)
    plt.scatter(red_reef_center.X(), red_reef_center.Y(), c=reef_colors[i])

branch_keys = list(FieldConstants.Reef.branchPositions[0].keys())

branch_colors = [color for color in reef_colors for _ in range(2)]

for i, reef_branch in enumerate(FieldConstants.Reef.branchPositions):
    plt.scatter(reef_branch[branch_keys[0]].X(), reef_branch[branch_keys[0]].Y(), c=branch_colors[i])
    
    red_branch = FieldConstants.flip_Translation2d(reef_branch[branch_keys[0]].translation())
    
    plt.scatter(red_branch.X(), red_branch.Y(), c=branch_colors[i])
    
plt.show()
