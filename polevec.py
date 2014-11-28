"""
Usage:
To create pole vector location form a joint selection (enter your desired offset as a number within the parenthesis):
    import polvec; polvec.getPoleVectorFromJoints( -your_offset_here- )

To create pole vector location form a ikHandle selection (enter your desired offset as a number within the parenthesis):
    import polvec; polvec.getPoleVectorFromIK( -your_offset_here- )

"""
__author__ = 'ssykes@cogswell.edu'

import pymel.core as pmc
from pymel.core.datatypes import Vector

def getPoleVectorPosition(joints, offset, curveGuide=True):
    """
    Creates locator with offset for the ideal pole vector control location
    Uses basic math to find the average position across all the joints and creates a vector
    from the average midpoint to the middle joint in the chain.
    """
    totalJoints = len(joints)    
    joints = map(pmc.nodetypes.Joint, joints)
    
    midpoint = Vector()
    midpoint.x = sum([jnt.getTranslation('world')[0] for jnt in joints])
    midpoint.y = sum([jnt.getTranslation('world')[1] for jnt in joints])
    midpoint.z = sum([jnt.getTranslation('world')[2] for jnt in joints])
    midpoint /= totalJoints

    if totalJoints % 2 == 0:
        first = totalJoints / 2
        second = first - 1

        midJointPos = Vector()
        midJointPos.x = joints[first].getTranslation('world')[0] + joints[second].getTranslation('world')[0]
        midJointPos.y = joints[first].getTranslation('world')[1] + joints[second].getTranslation('world')[1]
        midJointPos.z = joints[first].getTranslation('world')[2] + joints[second].getTranslation('world')[2]
        midJointPos /= 2
    else:
        midJointPos = Vector(pmc.xform(joints[totalJoints/2], q=True, worldSpace=True, translation=True))

    poleVector = (midJointPos - midpoint).normal()

    result = pmc.spaceLocator(n='loc_midchain_test')
    result.setTranslation((poleVector * offset) + midJointPos, 'world')

    if curveGuide:
        crv = pmc.curve(degree=1, point=[midpoint,(poleVector * 200) + midJointPos], k=[0, 1], n='curveGuide')

    return result

def getPoleVectorFromIK(offset):
    """
    Creates pole vector locator from a selected IK Handle
    """

    ikHandle = pmc.selected()[0]
    joints = pmc.ikHandle(ikHandle, q=True, jointList=True)
    joints.extend(pmc.listConnections(ikHandle.getEndEffector().translateX))

    loc = getPoleVectorPosition(joints, offset)
    loc.select(replace=True)

def getPoleVectorFromJoints(offset):
    """
    Creates pole vector locator from a selected set of joints
    """

    loc = getPoleVectorPosition(pmc.selected(), offset)
    loc.select(replace=True)