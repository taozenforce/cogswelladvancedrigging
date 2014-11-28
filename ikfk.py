from itertools import izip
import pymel.core as pmc

from hellamath import getPoleVectorPosition
from advutils import alignObjects

def makeIkFkJoints(joints, attribute=None, stretchy=False,
                   jointPrefix='rig', ikJointPrefix='ikj', fkJointPrefix='fkj'):
    """
    Creates 2 duplicate hierarchies from passed in joints
    (please make sure the list of joints are in same hierarchy)
    Blender is connected to attribute
    If stretchy is true, translation is connected for all the joints, except the root joint
    """

    startJoint = joints[0]
    rootRotation = startJoint.getRotation('world')
    rootTranslation = startJoint.getTranslation('world')

    # pymel bug, duplicating without renaming may result in some console warnings and errors
    # use a temp name until we learn the actual names for each joints
    fkJoints = pmc.duplicate(joints, parentOnly=True, name='fkTEMP')

    ## FK Joint setup

    # Use our prefix to uniquely rename duplicated joints based on the original copy
    # third parameter in jnt.replace allows me to limit the number of string replacements,
    # so I can avoid renaming joints containing 'right' in their name (I'm using 'rig' as my joint prefix)
    for jnt, fkj in izip(joints, fkJoints):
        fkj.rename(jnt.replace(jointPrefix, fkJointPrefix, 1))

    # Self grouping the duplicated FK root joint
    # Remember to not freeze transforms to ensure a proper connection to the original
    fkGrp = pmc.group(empty=True,
                      name='grp_fk_{0}_joints'.format(fkJoints[0].shortName()))
    fkGrp.setRotation(rootRotation, 'world')
    fkGrp.setTranslation(rootTranslation, 'world')

    # Could use a parent constrain here instead to the first parent
    # I prefer pointOrient for the extra control and future proofing
    parent = startJoint.firstParent2()
    if parent:
        pmc.parentConstraint(parent, fkGrp, maintainOffset=True)

    pmc.parent(fkJoints[0], fkGrp)

    ## IK Joint setup
    # Mostly the same code, this could be remade into a simpler function

    ikJoints = pmc.duplicate(joints, parentOnly=True, name='ikTEMP')

    for jnt, ikj in izip(joints, ikJoints):
        ikj.rename(jnt.replace(jointPrefix, ikJointPrefix, 1))

    ikGrp = pmc.group(empty=True,
                      name='grp_ik_{0}_joints'.format(ikJoints[0].shortName()))
    ikGrp.setRotation(rootRotation, 'world')
    ikGrp.setTranslation(rootTranslation, 'world')

    if parent:
        pmc.parentConstraint(parent, ikGrp, maintainOffset=True)

    pmc.parent(ikJoints[0], ikGrp)

    # we may have a attribute that goes between 0-1, 0-10 or even 0-100
    # Create remapValue node to help us remap the value, if needed
    outputAttr = None
    if attribute:
        if isinstance(attribute, basestring):
            outputAttr = pmc.Attribute(attribute)
        else:
            outputAttr = attribute

        attrMaxValue = outputAttr.getMax()
        if attrMaxValue > 1.0:
            attrNormalizeNode = pmc.shadingNode('remapValue', asUtility=True,
                name='rmv_{0}_ikfk_attr'.format(startJoint.shortName()))
            pmc.connectAttr(attribute, attrNormalizeNode.inputValue)
            attrNormalizeNode.inputMax.set(attrMaxValue)
            outputAttr = attrNormalizeNode.outValue

    # Connecting duplicate joint chains to original hierarchy
    # enumerate gives me an index, plus the joint to work with
    blendNodes = list()
    for i, jnt in enumerate(joints):
        fkJoint = fkJoints[i]
        ikJoint = ikJoints[i]

        # For this example, I connect my IKFK chains using blendColors nodes.
        # Using constraints instead is fine, but in place of this code,
        # you'll instead be connecting to the constraint's
        # weight values
        blender = pmc.shadingNode('blendColors', asUtility=True,
                                  name='bln_{0}_ikfk'.format(jnt.shortName()))
        blender.output.connect(jnt.rotate)

        fkJoint.rotate.connect(blender.color1)
        ikJoint.rotate.connect(blender.color2)
        blendNodes.append(blender)

        # Stretching joints by using X translation,
        # simpler to deal with and causes the least amount of headaches
        if stretchy is not None and jnt != startJoint:
            stretchyblender = pmc.shadingNode('blendColors', asUtility=True,
                                              name='bln_{0}_ikfk_stretch'.format(jnt.shortName()))

            fkJoint.translateX.connect(stretchyblender.color1R)
            ikJoint.translateX.connect(stretchyblender.color2R)

            stretchyblender.outputR.connect(jnt.translateX)
            blendNodes.append(stretchyblender)

    for bln in blendNodes:
        if outputAttr:
            outputAttr.connect(bln.blender)
        else:
            bln.blender.set(0)

    pmc.select(clear=True)
    return ikGrp, fkGrp, blendNodes

def matchFkToIk(fkControls, msgAttr='ikjoints', autoKey=True):
    """
    Matches fkControls to match the current pose of the underlying ik duplicate joint chains
    Finds the ik joints using a previously created message connection to the attribute msgAttr
    """

    ikJoints = None
    for ctl in fkControls:
        if pmc.hasAttr(ctl, msgAttr):
            ikJoints = pmc.listConnections('{0}.{1}'.format(ctl, msgAttr),
                                           destination=False, source=True)

            attr = pmc.listConnections('{0}.ikfk'.format(ctl), destination=False,
                                                source=True, plugs=True, scn=True)[0]
            switchControl = attr.node()
            switchAttr = attr.name(includeNode=False)
            break

    if autoKey:
        frameBeforeCurrent = pmc.currentTime(q=True) - 1
        pmc.setKeyframe(switchControl, attribute=switchAttr, time=frameBeforeCurrent,
                        value=0, outTangentType='step')
        pmc.setKeyframe(fkControls, attribute='rotate', time=frameBeforeCurrent,
                        outTangentType='step')

        pmc.setKeyframe(switchControl, attribute=switchAttr, value=1, outTangentType='step')

    for ikj, ctl in izip(ikJoints, fkControls):
        alignObjects(ctl, ikj, position=False, rotation=True)

    if autoKey:
        pmc.setKeyframe(fkControls, attribute='rotate')

    pmc.headsUpMessage('BAMF!')

def matchIkToFk(ikControl, ikPole, offset=100.0, msgAttr='fkjoints', autoKey=True):
    """
    Matches ikControl and ikPole Vector control to match the current pose of underlying
    fk duplicate joint chains
    Finds the fk joints using a previously created message connection to the attribute msgAttr
    """

    fkJoints = pmc.listConnections('{0}.{1}'.format(ikControl, msgAttr),
                                   destination=False, source=True)

    attr = pmc.listConnections('{0}.ikfk'.format(ikControl), destination=False,
                                        source=True, plugs=True, scn=True)[0]
    switchControl = attr.node()
    switchAttr = attr.name(includeNode=False)

    if autoKey:
        frameBeforeCurrent = pmc.currentTime(q=True) - 1
        pmc.setKeyframe(switchControl, attribute=switchAttr, time=frameBeforeCurrent,
                        value=1, outTangentType='step')
        pmc.setKeyframe([ikControl, ikPole], time=frameBeforeCurrent, outTangentType='step')

        pmc.setKeyframe(switchControl, attribute=switchAttr, value=0, outTangentType='step')

    alignObjects(ikControl, fkJoints[-1])
    loc = getPoleVectorPosition(fkJoints, offset, curveGuide=False)
    alignObjects(ikPole, loc, position=True, rotation=False)
    pmc.delete(loc)

    if autoKey:
        pmc.setKeyframe([ikControl, ikPole], time=frameBeforeCurrent)

    pmc.headsUpMessage('BAMF!')