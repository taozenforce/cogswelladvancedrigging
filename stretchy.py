import pymel.core as pmc


def makeStretchyClamp(normalizeNode, minStretch, maxStretch, name):
    clampNode = pmc.createNode('clamp', n='clp_{0}_stretch'.format(name))
    clampNode.minR.set(minStretch)
    clampNode.maxR.set(maxStretch)

    normalizeNode.outputX.connect(clampNode.inputR)
    return clampNode


def makeStretchyGlobalScale(globalScaleAttr, distance, name):
    globalScaleNode = pmc.createNode('multiplyDivide', n='mul_{0}_globalScaleStretch'.format(name))
    globalScaleNode.input2X.set(distance)
    pmc.connectAttr(globalScaleAttr, globalScaleNode.input1X)

    return globalScaleNode


def basicStretchyIk(ikHandle, stretchLimits=None, globalScaleAttr=None):
    """
    stretchLimits - tuple with the min and max values for scaling the joints. if set to None, no limits will be set
    """
    if isinstance(ikHandle, basestring):
        ikHandle = pmc.nodetypes.IkHandle(ikHandle)

    # Turn off snapping to avoid cycle error
    ikHandle.snapEnable.set(False)

    joints = ikHandle.getJointList()
    joints.extend(pmc.listConnections(ikHandle.getEndEffector().translateX))

    # generate locators
    # move onto appropriate positioning
    startLoc = pmc.spaceLocator(n='loc_{0}_stretchyStart'.format(ikHandle.shortName()))
    endLoc = pmc.spaceLocator(n='loc_{0}_stretchyEnd'.format(ikHandle.shortName()))

    pmc.pointConstraint(joints[0], startLoc, maintainOffset=False)
    pmc.pointConstraint(ikHandle, endLoc, maintainOffset=False)

    # create distance node and connect
    # Using createNode() prevents distNode from showing up in Hypershade's utility tab
    # May or may not be helpful to you, based on how tidy you want to keep the scene
    distNode = pmc.createNode('distanceBetween', n='dst_{0}_length'.format(ikHandle.shortName()))

    # Using locator's shape nodes to connect the worldPosition attribute
    startLoc.getShape().worldPosition[0].connect(distNode.point1)
    endLoc.getShape().worldPosition[0].connect(distNode.point2)

    # Get total distance using python's sum() function
    # Returns sum of all values in a list
    # List created inline using list comprehension
    distance = sum([abs(jnt.translateX.get()) for jnt in joints[1:]])

    # create divide node to divide current length
    normalizeNode = pmc.createNode('multiplyDivide', n='div_{0}_normalizer'.format(ikHandle.shortName()))
    distNode.distance.connect(normalizeNode.input1X)
    normalizeNode.operation.set(2)
    normalizeNode.input2X.set(distance)

    outputAttr = normalizeNode.outputX
    if stretchLimits:
        clampNode = makeStretchyClamp(normalizeNode=normalizeNode, minStretch=stretchLimits[0],
                                      maxStretch=stretchLimits[1], name=ikHandle.shortName())
        outputAttr = clampNode.outputR

    # If rig has global scale, create additional nodes to preserve length
    if globalScaleAttr:
        globalScaleNode = makeStretchyGlobalScale(globalScaleAttr, distance, name=ikHandle.shortName())
        globalScaleNode.outputX.connect(normalizeNode.input2X)

    # multiply node to scale each joint's translateX
    # connect multiply nodes to joints
    for jnt in joints[1:]:
        scaleNode = pmc.createNode('multiplyDivide', n='mul_{0}_stretch'.format(jnt.shortName()))

        outputAttr.connect(scaleNode.input1X)
        scaleNode.input2X.set(jnt.translateX.get())
        scaleNode.outputX.connect(jnt.translateX)

    return [startLoc, endLoc]


def stretchySplineIk(ikHndl, useScale=False, stretchLimits=None, globalScaleAttr=None, usePointOnCurve=False):
    """
    Builds makes selected SplineIK Stretchy using either joint scale
       or translate, as passed in parameter.
       If usePointOnCurve is False, calculates length of chain using curve Info, which doesn't
       give as accurate a result but invlolves less nodes and connections
       If true, locators are created to calculate the true length to stretch spline IK joints
    """

    if isinstance(ikHndl, basestring):
        ikHndl = pmc.nodetypes.IkHandle(ikHndl)

    # Need to find effector and start/end joint of this IK handle
    curve = pmc.listConnections((ikHndl + '.inCurve'), sh=True)[0]
    effector = pmc.listConnections((ikHndl + '.endEffector'))[0]
    endJoint = pmc.listConnections((effector + '.tx'))[0]
    joints = ikHndl.getJointList()

    # Building proper nodes
    if usePointOnCurve:
        paramStep = 1.0 / len(joints)
        locators = list()
        currentStep = 0.0
        i = 0
        while currentStep <= 1.0:
            onCrv = pmc.createNode('pointOnCurveInfo', name='pci_{0}_stretchy{1}'.format(ikHndl.shortName(), i))
            onCrv.turnOnPercentage.set(1)
            onCrv.parameter.set(currentStep)
            curve.worldSpace[0].connect(onCrv.inputCurve)

            loc = pmc.spaceLocator(n='loc_{0}_stretch{1}'.format(ikHndl.shortName(), i))
            onCrv.result.position.connect(loc.translate)
            locators.append(loc)
            currentStep += paramStep
            i += 1

        totalDistNode = pmc.createNode('plusMinusAverage', n='pls_{0}_totallength'.format(ikHndl.shortName()))

        for i, loc in enumerate(locators[:-2]):
            distNode = pmc.createNode('distanceBetween', n='dst_{0}_sublength{1}'.format(ikHndl.shortName(), i))

            # Using locator's shape nodes to connect the worldPosition attribute
            loc.getShape().worldPosition[0].connect(distNode.point1)
            locators[i + 1].getShape().worldPosition[0].connect(distNode.point2)
            distNode.distance.connect(totalDistNode.input1D[i])

        normalizeNode = pmc.createNode('multiplyDivide', n='div_{0}_normalizer'.format(ikHndl.shortName()))
        normalizeNode.operation.set(2)
        totalDistNode.output1D.connect(normalizeNode.input1X)
        normalizeNode.input2X.set(totalDistNode.output1D.get())

    else:
        cvInfo = pmc.createNode('curveInfo')
        curve.worldSpace[0].connect(cvInfo.inputCurve)
        normalizeNode = pmc.createNode('multiplyDivide', n='div_{0}_normalizer'.format(ikHndl.shortName()))
        cvInfo.arcLength.connect(normalizeNode.input1X)
        normalizeNode.operation.set(2)
        normalizeNode.input2X.set(cvInfo.arcLength.get())

    outputAttr = normalizeNode.outputX
    if stretchLimits:
        clampNode = makeStretchyClamp(normalizeNode=normalizeNode, minStretch=stretchLimits[0],
                                      maxStretch=stretchLimits[1], name=ikHndl.shortName())
        outputAttr = clampNode.outputR

    # If rig has global scale, create additional nodes to preserve length
    if globalScaleAttr:
        globalScaleNode = makeStretchyGlobalScale(globalScaleAttr, normalizeNode.input2X.get(), name=ikHndl.shortName())
        globalScaleNode.outputX.connect(normalizeNode.input2X)

    uniformNode = None

    if useScale is False:
        joints = joints[1:]
        joints.append(endJoint)
        isUniformChain = all(jnt.tx.get() == endJoint.tx.get() for jnt in joints)

        if isUniformChain:
            uniformNode = pmc.createNode('multiplyDivide', n='mul_{0}_stretch'.format(ikHndl.shortName()))
            outputAttr.connect(uniformNode.input1X)
            uniformNode.input2X.set(endJoint.tx.get())

            uniformNode.outputX.connect()

    for jnt in joints:
        if useScale:
            outputAttr.connect(jnt.scaleX)
        elif uniformNode:
            outputAttr.connect(jnt.translateX)
        else:
            stretchNode = pmc.createNode('multiplyDivide', n=('mul_{0}_stretchy'.format(jnt)))
            outputAttr.connect(stretchNode.input1X)
            stretchNode.input2X.set(jnt.translateX.get())
            stretchNode.outputX.connect(jnt.translateX)