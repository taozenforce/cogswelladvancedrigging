__author__ = 'Serge'
import pymel.core as pmc

import advutils as adv

from splitter import Splitter
from stretchy import stretchySplineIk


def squishySplineIk(startLoc, endLoc):
    ikJoints = list()
    startJoint = pmc.createNode('joint')
    adv.alignObjects(startJoint, startLoc)

    endJoint = pmc.createNode('joint')
    adv.alignObjects(endJoint, endLoc)
    pmc.parent(endJoint, startJoint)

    startJoint.orientJoint('xzy', secondaryAxisOrient='zup')
    pmc.makeIdentity(endJoint, apply=True, jointOrient=True)

    Splitter.doSplit(startJoint, 11)

    ikJoints.append(startJoint)
    ikJoints.extend(reversed(startJoint.getChildren(ad=True, type='joint')))

    for i, ikj in enumerate(ikJoints):
        ikj.radius.set(2)
        ikj.rename('ikj_spine{0:d}'.format(i))

    # Create second set of joints
    rigJoints = adv.makeDuplicateJoints(joints=ikJoints, search='ikj_', replace='jnt_', connectBone=False)
    print rigJoints
    # HACK I haven't figured out how to create SDK nodes procedurall, so making some dummy locs to make the curve I need
    a = pmc.createNode('transform')
    b = pmc.createNode('transform')

    pmc.setDrivenKeyframe(b.ty, currentDriver=a.ty, driverValue=0, value=0,
                          inTangentType='flat', outTangentType='flat')
    pmc.setDrivenKeyframe(b.ty, currentDriver=a.ty, driverValue=3, value=2.5,
                          inTangentType='flat', outTangentType='flat')
    pmc.setDrivenKeyframe(b.ty, currentDriver=a.ty, driverValue=10, value=0,
                          inTangentType='flat', outTangentType='flat')

    animCurve = a.ty.listConnections()[0]
    a.ty.disconnect(animCurve.input)
    animCurve.output.disconnect(b.ty)
    animCurve.rename('squash_ramp')
    pmc.delete(a, b)

    animControls = dict()
    animControls['lower_spine'] = adv.makeControlNode('ctl_lower_spine', targetObject=rigJoints[2], alignRotation=False)
    animControls['middle_spine'] = adv.makeControlNode('ctl_middle_spine')
    animControls['upper_spine'] = adv.makeControlNode('ctl_upper_spine', targetObject=rigJoints[-2],
                                                      alignRotation=False)

    animControls['lower_spine'][0].rotateOrder.set(adv.ROO_YXZ)
    animControls['middle_spine'][0].rotateOrder.set(adv.ROO_YXZ)
    animControls['upper_spine'][0].rotateOrder.set(adv.ROO_YXZ)

    pmc.pointConstraint(animControls['lower_spine'][0], animControls['upper_spine'][0],
                        animControls['middle_spine'][1], mo=False)

    pmc.orientConstraint(animControls['lower_spine'][0], animControls['upper_spine'][0],
                         animControls['middle_spine'][1], mo=False)

    splineIk = pmc.ikHandle(sj=ikJoints[0], ee=ikJoints[-1], sol='ikSplineSolver', parentCurve=False,
                            createCurve=True, simplifyCurve=True, numSpans=2, rootOnCurve=False, n='sik_spine')

    splineIkHandle = splineIk[0]
    spline = splineIk[2]

    clusterJoints = list()
    clusterJoints.append(pmc.createNode('joint', n='clj_spine0'))
    adv.alignObjects(clusterJoints[-1], animControls['lower_spine'][0])
    pmc.parent(clusterJoints[-1], animControls['lower_spine'][0])

    clusterJoints.append(pmc.createNode('joint', n='clj_spine1'))
    adv.alignObjects(clusterJoints[-1], animControls['middle_spine'][0])
    pmc.parent(clusterJoints[-1], animControls['middle_spine'][0])

    clusterJoints.append(pmc.createNode('joint', n='clj_spine2'))
    adv.alignObjects(clusterJoints[-1], animControls['upper_spine'][0])
    pmc.parent(clusterJoints[-1], animControls['upper_spine'][0])

    pmc.skinCluster(clusterJoints, spline, maximumInfluences=3)

    pmc.parentConstraint()

    for clj in clusterJoints:
        clj.radius.set(3)

    splineIkHandle.dTwistControlEnable.set(1)
    splineIkHandle.dWorldUpType.set(4)
    splineIkHandle.dWorldUpAxis.set(0)
    splineIkHandle.dWorldUpVector.set([0.0, 0.0, 1.0])
    splineIkHandle.dWorldUpVectorEnd.set([0.0, 0.0, 1.0])

    animControls['lower_spine'][0].worldMatrix[0].connect(splineIkHandle.dWorldUpMatrix)
    animControls['upper_spine'][0].worldMatrix[0].connect(splineIkHandle.dWorldUpMatrixEnd)

    stretchySplineIk(splineIkHandle, useScale=True, globalScaleAttr='ctl_main.size')

    for i, jnt in enumerate(rigJoints):
        preTransform = adv.zeroOut(jnt, 'pre')

        ikNode = adv.zeroOut(jnt, 'hlp_ik')
        pmc.pointConstraint(ikJoints[i], ikNode)
        pmc.orientConstraint(ikJoints[i], ikNode)

        twistNode = adv.zeroOut(jnt, 'hlp_twist')


    # Make sure inverse scale is connected

# animCurve = pmc.PyNode('ctl_spine0_volume')
# invScale = pmc.PyNode('div_inverse_scale')
# for i, jnt in enumerate(pmc.selected()):
#     cache = pmc.createNode('frameCache', n='frm_{0}'.format(jnt.shortName()))
#     animCurve.output.connect(cache.stream)
#     cache.varyTime.set(i)
#
#     pow = pmc.createNode('multiplyDivide', n='pow_{0}'.format(jnt.shortName()))
#     pow.operation.set(3)
#     invScale.outputX.connect(pow.input1X)
#     cache.varying.connect(pow.input2X)
#     pow.outputX.connect(jnt.scaleY)
#     pow.outputX.connect(jnt.scaleZ)

    return ikJoints