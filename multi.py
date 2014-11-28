from itertools import izip
import pymel.core as pmc

from support import align, getAttribute


def makeMultiConstraint(targets, source, controller, attrName='currentSpace',
                        enumNames=None, addLocatorSpace=True, translation=True, rotation=True, maintainOffset=False):
    """
    creates a constraint between all to the targets to the passed in source.

    """

    targets = map(pmc.PyNode, targets)
    source = pmc.PyNode(source)
    controller = pmc.PyNode(controller)
    loc = None

    if enumNames is None:
        enumNames = map(str, targets)

    if addLocatorSpace:
        loc = pmc.spaceLocator(n='loc_follow_' + source.shortName())
        align.alignObjects(loc, source)
        targets.append(loc)
        enumNames.append('locator')

    skipTranslate = 'none'
    if not translation:
        skipTranslate = ['x', 'y', 'z']

    skipRotate = 'none'
    if not rotation:
        skipRotate = ['x', 'y', 'z']

    constraints = list()
    for tgt, spaceName in izip(targets, enumNames):
        enumAttr = getAttribute(controller, attrName,
            at='enum', enumName=spaceName, keyable=True, longName=attrName)

        spaceEnums = enumAttr.getEnums().keys()
        if spaceName not in spaceEnums:
            spaceEnums.append(spaceName)
            enumAttr.setEnums(spaceEnums)

        index = enumAttr.getEnums().value(spaceName)

        if tgt is loc:
            tgtTransform = loc
        else:
            tgtTransform = pmc.createNode('transform',
                                          n='tgt_{0}_to_{1}'.format(source.shortName(), tgt.shortName()))
            align.alignObjects(tgtTransform, source)
            pmc.parentConstraint(tgt, tgtTransform, maintainOffset=maintainOffset)

        orient = pmc.orientConstraint(tgtTransform, source, skip=skipRotate, maintainOffset=maintainOffset)
        orient.interpType.set(2)
        constraints.append(orient)

        point = pmc.pointConstraint(tgtTransform, source, skip=skipTranslate, maintainOffset=maintainOffset)
        constraints.append(point)

        orientWeightList = orient.getWeightAliasList()
        orientWeightAttr = orientWeightList[-1]

        pointWeightList = point.getWeightAliasList()
        pointWeightAttr = pointWeightList[-1]

        nodeName = 'con_{0}_to_{1}_weight_{2:d}'.format(source.shortName(), tgt.shortName(), index)

        weightConditionNode = pmc.createNode('condition', n=nodeName)

        enumAttr.connect(weightConditionNode.firstTerm)

        weightConditionNode.secondTerm.set(index)
        weightConditionNode.colorIfTrueR.set(1)
        weightConditionNode.colorIfFalseR.set(0)

        weightConditionNode.outColorR.connect(pointWeightAttr)
        weightConditionNode.outColorR.connect(orientWeightAttr)

    return loc