"""
@author: Sergio Sykes - Cogswell Polytechnical College 2014

Example code for a modular rigging system. This code was tested in the creation of goldie and daniel
"""

import pymel.core as pmc

from advutils import getAttribute, alignObjects, makeControlNode, ROO_XZY, ROO_YXZ


def makePoleVectorLine(startObj, endObj, parent=None):
    curve = pmc.curve(degree=1, point=[(0, 0, 0), (0, 0, 1)], knot=range(2),
                       name='spl_{0}To{1}_poleLine'.format(startObj, endObj))

    curveShape = pmc.listRelatives(curve, s=True, f=True)[0]
    startClu = pmc.cluster(curveShape + '.cv[0]', relative=True,
                            name=curve.replace('spl_', 'clu_', 1) + '_start')[1]
    endClu = pmc.cluster(curveShape + '.cv[1]', relative=True,
                          name=curve.replace('spl_', 'clu_', 1) + '_end')[1]

    pmc.pointConstraint(startObj, startClu)
    pmc.pointConstraint(endObj, endClu)

    for node in curve, startClu, endClu:
        pmc.setAttr(node + '.inheritsTransform', 0)

    pmc.setAttr(curveShape + '.overrideEnabled', 1)
    pmc.setAttr(curveShape + '.overrideDisplayType', 1)
    pmc.setAttr(startClu + '.visibility', 0)
    pmc.setAttr(endClu + '.visibility', 0)

    if parent:
        pmc.parent(startClu, endClu, curve, parent)

    return curve


def makePoleVectorControlFromHandle(name, ikHandle, offset=10, parent=None):
    """
    Creates pole position using the poleVector attribute from the specified ikHandle
    """
    polePosition = [i * offset for i in pmc.getAttr(ikHandle + '.poleVector')]

    ctrl, preTransform = makeControlNode(name)

    # move to start joint location
    pmc.xform(preTransform, worldSpace=True,
               translation=pmc.xform(pmc.ikHandle(ikHandle, q=True, startJoint=True), q=True, worldSpace=True,
                                      translation=True))

    # offset along pole vector (move relative)
    pmc.xform(preTransform, relative=True, objectSpace=True, translation=polePosition)

    midJoint = pmc.ikHandle(ikHandle, q=True, jointList=True)
    curve = makePoleVectorLine(midJoint[len(midJoint) / 2], ctrl, parent)

    pmc.connectAttr(ctrl + '.visibility', curve + '.visibility')

    if parent:
        pmc.parent(preTransform, parent)

    return ctrl, curve


class Rigging(object):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        self._name = name
        self._joints = joints
        self._switchboard = switchboard
        self._mainControl = mainControl
        self._rigControls = dict()
        self.lockAttrs = list()
        self.transform = pmc.group(empty=True, name='grp_{0}_rig'.format(self._name))

        if parent is None:
            locName = 'loc_{0}_parentMe'.format(self._name)
            if pmc.objExists(locName):
                pmc.delete(locName)
            self._parent = pmc.spaceLocator(n=locName)
            alignObjects([self._parent], self._joints[0])
            pmc.parent(self._parent, self.transform)
        else:
            self._parent = parent

        if pmc.objExists('ctl_visibility'):
            visAttr = getAttribute('ctl_visibility', self._name, at='short', min=0, max=1, dv=1)
            pmc.setAttr(visAttr, edit=True, channelBox=True)
            pmc.connectAttr(visAttr, self.transform + '.v')

    def lockAndHide(self, lock):
        for at in self.lockAttrs:
            pmc.setAttr(at, lock=lock)
            pmc.setAttr(at, keyable=not lock)
            pmc.setAttr(at, channelBox=not lock)

    def makeJointSystems(self, prefix, isolation=True, makeConstraints=True):
        joints = pmc.duplicate(self._joints, parentOnly=True, name='{0}_TEMP'.format(prefix))
        joints[0] = pmc.rename(joints[0], self._joints[0].replace('rig_', prefix + '_', 1))

        i = 1
        for jnt, dupe in zip(self._joints[1:], joints[1:]):
            joints[i] = pmc.rename(dupe, jnt.replace('rig_', prefix + '_', 1))
            i += 1

        grp = pmc.group(empty=True, name='grp_{0}_{1}_joints'.format(prefix, self._name))
        root_rotation = pmc.xform(joints[0], q=True, ws=True, rotation=True)
        root_position = pmc.xform(joints[0], q=True, ws=True, translation=True)

        pmc.xform(grp, a=True, ws=True, rotation=root_rotation, translation=root_position)

        if makeConstraints:
            if isolation:
                pmc.pointConstraint(self._joints[0], grp, maintainOffset=False)
                pmc.orientConstraint(self._parent, grp, maintainOffset=True)
            else:
                pmc.parentConstraint()

        pmc.parent(joints[0], grp)

        return joints

    @staticmethod
    def connectJointChains(joints, blendAttr, stretchy=True, useConstraints=False):
        """
        Connect related IK/FK chains to joint chain between baseStartJoint and endJoint
        blendColors nodes are creaed to switch between IK/FK and are conencted to the blendAttr
        if stretchy is True, translationX channels are also blended for later stretchy setups
        isReversed - True changes 0 to fk, 1 to ik
        """

        for jnt in joints:
            fkjoint = jnt.replace('rig_', 'fkj_', 1)
            ikjoint = jnt.replace('rig_', 'ikj_', 1)

            if useConstraints:
                point = pmc.pointConstraint(ikjoint, fkjoint, jnt, mo=False)
                orient = pmc.orientConstraint(ikjoint, fkjoint, jnt, mo=False)

                ikreverse = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_ikfk'.format(jnt))

                orientWeightList = pmc.orientConstraint(orient, q=True, weightAliasList=True)
                pointWeightList = pmc.pointConstraint(point, q=True, weightAliasList=True)

                pmc.connectAttr(blendAttr, orientWeightList[1])
                pmc.connectAttr(blendAttr, pointWeightList[1])
                pmc.connectAttr(blendAttr, ikreverse + '.inputX')
                pmc.connectAttr(ikreverse + '.outputX', orientWeightList[0])
                pmc.connectAttr(ikreverse + '.outputX', pointWeightList[0])

                if stretchy:
                    scale = pmc.scaleConstraint(ikjoint, fkjoint, jnt, mo=False)
                    scaleWeightList = pmc.scaleConstraint(point, q=True, weightAliasList=True)
                    pmc.connectAttr(blendAttr, scaleWeightList[1])
                    pmc.connectAttr(ikreverse + '.outputX', scaleWeightList[0])

            else:
                rotationblender = pmc.shadingNode('blendColors', asUtility=True, name='bln_{0}_ikfk_rot'.format(jnt))

                pmc.connectAttr(fkjoint + '.rotate', rotationblender + '.color1')
                pmc.connectAttr(ikjoint + '.rotate', rotationblender + '.color2')

                pmc.connectAttr(blendAttr, rotationblender + '.blender')
                pmc.connectAttr(rotationblender + '.output', jnt + '.rotate')

                if stretchy and jnt != joints[0]:
                    node = pmc.shadingNode('blendColors', asUtility=True, name='bln_{0}_ikfk_scale'.format(jnt))

                    pmc.connectAttr(fkjoint + '.tx', node + '.color1R')
                    pmc.connectAttr(ikjoint + '.tx', node + '.color2R')

                    pmc.connectAttr(blendAttr, node + '.blender')
                    pmc.connectAttr(node + '.outputR', jnt + '.tx')

    def makeOrientSwitchNodes(self, joint, preTransform, name=None):
        """
        Creates a orient constraint on the specified preTransform.
        Switches between following the world and following the
        self.parent of the current Rig Module. Joint specifies the which object to apply
        the pointConstraint to the preTransform
        """

        if name is None:
            name = self._name

        pmc.pointConstraint(joint, preTransform)

        parentTarget = pmc.group(empty=True, name='tgt_{0}_local'.format(name))
        worldTarget = pmc.group(empty=True, name='tgt_{0}_world'.format(name))
        alignObjects([parentTarget, worldTarget], preTransform)

        pmc.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        pmc.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        constraint = pmc.orientConstraint(parentTarget, preTransform)
        pmc.orientConstraint(worldTarget, preTransform)
        pmc.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, name + '_isolation',
                                  min=0, max=1, defaultValue=1, keyable=True)

        revAttrNode = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_isolation'.format(name))

        orientWeightList = pmc.orientConstraint(constraint, q=True, weightAliasList=True)
        pmc.connectAttr(orientAttr, orientWeightList[1])
        pmc.connectAttr(orientAttr, revAttrNode + '.inputX')
        pmc.connectAttr(revAttrNode + '.outputX', orientWeightList[0])

        return [parentTarget, worldTarget]

    def makeNoFlipHelper(self, ikHandle, aimVector):
        helper = pmc.group(empty=True, name=ikHandle.replace('ikh_', 'hlp_ik_') + '_noflipper')
        startJoint = pmc.ikHandle(ikHandle, q=True, sj=True)

        pmc.pointConstraint(startJoint, helper, maintainOffset=False)
        pmc.aimConstraint(ikHandle, helper, aimVector=aimVector, upVector=aimVector, worldUpType='objectrotation',
                           worldUpVector=(0, 1, 0), worldUpObject=self._mainControl)
        return helper


class RiggingLeg(Rigging):
    BALL_ROLL_ATTR_NAME = 'heelRoll'
    TOE_ROLL_ATTR_NAME = 'toeRoll'
    TOE_PIVOT_ATTR_NAME = 'toePivotX'
    TOE_YAW_ATTR_NAME = 'toePivotY'
    HEEL_PIVOT_ATTR_NAME = 'heelPivotX'
    FOOT_BANK_ATTR_NAME = 'footBank'

    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, noFlipVector=None):
        super(RiggingLeg, self).__init__(name, joints, parent, mainControl, switchboard)

        self._noflipvector = noFlipVector
        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(self._switchboard, self._name)
        self._fkjoints = self.makeJointSystems('fkj')
        self._ikjoints = self.makeJointSystems('ikj')
        self.connectJointChains(joints=self._joints[:-1], blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name))

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        fkjointgrp = pmc.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = pmc.listRelatives(self._ikjoints[0], parent=True)[0]

        pmc.parent(fkrig, fkjointgrp, ikrig, ikjointgrp, self.transform)

        self.lockAndHide(True)

    def makeFkRig(self):
        """
        Fk setup for leg
        Assumes user selects hip, knee, ankle, and ball joint; rigs with control curves
        returns created controls as PyNodes
        """
        mainGroup = pmc.group(empty=True, name='grp_fk_{0}_rig'.format(self._name))

        jnts = {'hip': self._fkjoints[0], 'knee': self._fkjoints[1], 'ankle': self._fkjoints[2],
                'ball': self._fkjoints[3], 'toe': self._fkjoints[4]}

        self._rigControls['fk_hip'], hipPreTransform = makeControlNode(name='ctl_fk_{0}_hip'.format(self._name),
                                                                       targetObject=jnts['hip'])
        self._rigControls['fk_knee'], kneePreTransform = makeControlNode(name='ctl_fk_{0}_knee'.format(self._name),
                                                                         targetObject=jnts['knee'])
        self._rigControls['fk_ankle'], anklePreTransform = makeControlNode(name='ctl_fk_{0}_ankle'.format(self._name),
                                                                           targetObject=jnts['ankle'])
        self._rigControls['fk_ball'], ballPreTransform = makeControlNode(name='ctl_fk_{0}_ball'.format(self._name),
                                                                         targetObject=jnts['ball'])

        pmc.pointConstraint(jnts['hip'], hipPreTransform)

        parentTarget = pmc.group(empty=True, name='tgt_fk_{0}_local'.format(self._name))
        worldTarget = pmc.group(empty=True, name='tgt_fk_{0}_world'.format(self._name))
        alignObjects([parentTarget, worldTarget], hipPreTransform)

        pmc.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        pmc.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        constraint = pmc.orientConstraint(parentTarget, hipPreTransform)
        pmc.orientConstraint(worldTarget, hipPreTransform)
        pmc.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, self._name + '_fk_isolation',
                                  min=0, max=1, defaultValue=1, keyable=True)

        revAttrNode = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_fk_isolation'.format(self._name))

        orientWeightList = pmc.orientConstraint(constraint, q=True, weightAliasList=True)
        pmc.connectAttr(orientAttr, orientWeightList[1])
        pmc.connectAttr(orientAttr, revAttrNode + '.inputX')
        pmc.connectAttr(revAttrNode + '.outputX', orientWeightList[0])

        pmc.parent(parentTarget, worldTarget, mainGroup)

        pmc.orientConstraint(self._rigControls['fk_ball'], jnts['ball'])
        pmc.orientConstraint(self._rigControls['fk_ankle'], jnts['ankle'])
        pmc.orientConstraint(self._rigControls['fk_knee'], jnts['knee'])
        pmc.orientConstraint(self._rigControls['fk_hip'], jnts['hip'])

        pmc.parent(ballPreTransform, self._rigControls['fk_ankle'], absolute=True)
        pmc.parent(anklePreTransform, self._rigControls['fk_knee'], absolute=True)
        pmc.parent(kneePreTransform, self._rigControls['fk_hip'])

        pmc.parent(hipPreTransform, mainGroup)

        if self._switchAttr:
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_ankle'] + '.visibility')
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_ball'] + '.visibility')
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_knee'] + '.visibility')
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_hip'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_ankle'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_ball'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_hip'] + '.visibility')

        return mainGroup

    def makeIkRig(self):
        mainGroup = pmc.group(empty=True, name='grp_ik_{0}_rig'.format(self._name))

        jnts = {'hip': self._ikjoints[0], 'knee': self._ikjoints[1], 'ankle': self._ikjoints[2],
                'ball': self._ikjoints[3], 'toe': self._ikjoints[4]}

        self._rigControls['ik_leg'], legPreTransform = makeControlNode(name='ctl_ik_{0}'.format(self._name),
                                                                       targetObject=jnts['ankle'], alignRotation=False)

        pmc.setAttr(self._rigControls['ik_leg'] + '.rotateOrder', ROO_XZY)

        legHandle = pmc.ikHandle(sj=jnts['hip'], ee=jnts['ankle'], sol='ikRPsolver',
                                  n='ikh_{0}_leg'.format(self._name))[0]
        ballHandle = pmc.ikHandle(sj=jnts['ankle'], ee=jnts['ball'], sol='ikSCsolver',
                                   n='ikh_{0}_ball'.format(self._name))[0]
        toeHandle = pmc.ikHandle(sj=jnts['ball'], ee=jnts['toe'], sol='ikSCsolver',
                                  n='ikh_{0}_toe'.format(self._name))[0]

        self._rigControls['ik_knee'], poleLine = makePoleVectorControlFromHandle('ctl_ik_{0}_pole'.format(self._name),
                                                                                 legHandle, parent=mainGroup)

        pmc.poleVectorConstraint(self._rigControls['ik_knee'], legHandle)
        pmc.parent(legHandle, ballHandle, toeHandle, self._rigControls['ik_leg'])

        # setup reverse foot nodes
        self._rigControls['ik_toe'], toePreTransform = makeControlNode(name='ctl_ik_{0}_toeRoll'.format(self._name),
                                                                       targetObject=jnts['ball'])
        toeRollNode = pmc.group(toePreTransform, name='hlp_ik_{0}_toeRoll'.format(self._name))
        ballRollNode = pmc.group(empty=True, name='hlp_ik_{0}_ballRoll'.format(self._name))
        toePivotNode = pmc.group(empty=True, name='hlp_ik_{0}_toePivot'.format(self._name))
        toeYawNode = pmc.group(empty=True, name='hlp_ik_{0}_toeYaw'.format(self._name))
        heelPivotNode = pmc.group(empty=True, name='hlp_ik_{0}_heelPivot'.format(self._name))
        footBankInNode = pmc.group(empty=True, name='hlp_ik_{0}_footBankIn'.format(self._name))
        footBankOutNode = pmc.group(empty=True, name='hlp_ik_{0}_footBankOut'.format(self._name))

        # positioning
        pmc.xform([ballRollNode, toeYawNode], worldSpace=True,
                   translation=pmc.xform(jnts['ball'], q=True, ws=True, t=True))

        toePivotPosition = pmc.xform(jnts['toe'], q=True, worldSpace=True, translation=True)
        toePivotPosition[1] = 0.0
        pmc.xform(toePivotNode, worldSpace=True, translation=toePivotPosition)

        # Heel pivot and foot bank will vary based on geometry, create a locator that can adjust the pivot
        heelLocator = pmc.spaceLocator(n='loc_ik_{0}_heelPivot'.format(self._name))
        footBankInLocator = pmc.spaceLocator(n='loc_ik_{0}_footBankIn'.format(self._name))
        footBankOutLocator = pmc.spaceLocator(n='loc_ik_{0}_footBankOut'.format(self._name))
        pmc.xform((heelPivotNode, heelLocator, footBankInNode, footBankOutNode, footBankInLocator, footBankOutLocator),
                   worldSpace=True, translation=pmc.xform(jnts['ankle'], q=True, ws=True, t=True))

        pmc.parent(toeRollNode, ballRollNode, toeYawNode)
        pmc.parent(toeYawNode, toePivotNode)
        pmc.parent(toePivotNode, heelPivotNode)
        pmc.parent(heelPivotNode, footBankInNode)
        pmc.parent(footBankInNode, footBankOutNode)
        pmc.parent(footBankOutNode, heelLocator, footBankInLocator, footBankOutLocator, self._rigControls['ik_leg'])

        pmc.makeIdentity((heelLocator, footBankInLocator, footBankOutLocator), apply=True)
        pmc.connectAttr(heelLocator + '.translate', heelPivotNode + '.rotatePivot')
        pmc.connectAttr(footBankInLocator + '.translate', footBankInNode + '.rotatePivot')
        pmc.connectAttr(footBankOutLocator + '.translate', footBankOutNode + '.rotatePivot')

        # Add and connect foot roll attributes
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.TOE_ROLL_ATTR_NAME, softMinValue=-10, softMaxValue=10,
                     defaultValue=0, keyable=True)
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.BALL_ROLL_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.TOE_PIVOT_ATTR_NAME, min=0, max=10, defaultValue=0,
                     keyable=True)
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.TOE_YAW_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.HEEL_PIVOT_ATTR_NAME, min=0, max=10, defaultValue=0,
                     keyable=True)
        pmc.addAttr(self._rigControls['ik_leg'], ln=self.FOOT_BANK_ATTR_NAME, min=-10, max=10, defaultValue=0,
                     keyable=True)

        toeRollMultiply = pmc.shadingNode('multiplyDivide', asUtility=True, n='mul_ik_{0}_toeRoll'.format(self._name))
        ballRollMultiply = pmc.shadingNode('multiplyDivide', asUtility=True,
                                            n='mul_ik_{0}_ballRoll'.format(self._name))
        toePivotMultiply = pmc.shadingNode('multiplyDivide', asUtility=True,
                                            n='mul_ik_{0}_toePivot'.format(self._name))
        toeYawMultiply = pmc.shadingNode('multiplyDivide', asUtility=True, n='mul_ik_{0}_toeYaw'.format(self._name))
        heelPivotMultiply = pmc.shadingNode('multiplyDivide', asUtility=True,
                                             n='mul_ik_{0}_heelPivot'.format(self._name))

        footBankRemap = pmc.shadingNode('remapValue', asUtility=True,
                                         n='rmv_ik_{0}_footBank'.format(self._name))
        footBankClamp = pmc.shadingNode('clamp', asUtility=True,
                                         n='clp_ik_{0}_footBank'.format(self._name))

        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_ROLL_ATTR_NAME),
                         toeRollMultiply + '.input1X')
        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.BALL_ROLL_ATTR_NAME),
                         ballRollMultiply + '.input1X')
        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_PIVOT_ATTR_NAME),
                         toePivotMultiply + '.input1X')
        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.TOE_YAW_ATTR_NAME),
                         toeYawMultiply + '.input1Y')
        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.HEEL_PIVOT_ATTR_NAME),
                         heelPivotMultiply + '.input1X')

        pmc.connectAttr('{0}.{1}'.format(self._rigControls['ik_leg'], self.FOOT_BANK_ATTR_NAME),
                         footBankRemap + '.inputValue')
        pmc.connectAttr(footBankRemap + '.outValue', footBankClamp + '.inputR')
        pmc.connectAttr(footBankRemap + '.outValue', footBankClamp + '.inputG')

        pmc.setAttr(toeRollMultiply + '.input2X', -9.5)
        pmc.setAttr(ballRollMultiply + '.input2X', 9.5)
        pmc.setAttr(toePivotMultiply + '.input2X', 9.5)
        pmc.setAttr(toeYawMultiply + '.input2Y', 9.5)
        pmc.setAttr(heelPivotMultiply + '.input2X', -11.0)
        pmc.setAttr(footBankRemap + '.inputMin', -10.0)
        pmc.setAttr(footBankRemap + '.inputMax', 10.0)
        pmc.setAttr(footBankRemap + '.outputMin', -90.0)
        pmc.setAttr(footBankRemap + '.outputMax', 90.0)
        pmc.setAttr(footBankClamp + '.maxR', 90.0)
        pmc.setAttr(footBankClamp + '.minG', -90.0)

        pmc.connectAttr(toeRollMultiply + '.outputX', toeRollNode + '.rotateX')
        pmc.connectAttr(ballRollMultiply + '.outputX', ballRollNode + '.rotateX')
        pmc.connectAttr(toePivotMultiply + '.outputX', toePivotNode + '.rotateX')
        pmc.connectAttr(toeYawMultiply + '.outputY', toeYawNode + '.rotateY')
        pmc.connectAttr(heelPivotMultiply + '.outputX', heelPivotNode + '.rotateX')
        pmc.connectAttr(footBankClamp + '.outputR', footBankInNode + '.rotateZ')
        pmc.connectAttr(footBankClamp + '.outputG', footBankOutNode + '.rotateZ')

        pmc.parent(toeHandle, self._rigControls['ik_toe'])
        pmc.parent(ballHandle, toeRollNode)
        pmc.parent(legHandle, ballRollNode)

        # parent to a group aligned to ankle joint, that is constrained only to y orientation
        kneeToFootHelper = pmc.group(empty=True, name='hlp_{0}_knee_to_foot'.format(self._name))

        alignObjects([kneeToFootHelper, ], self._rigControls['ik_leg'])
        pmc.parentConstraint(self._rigControls['ik_leg'], kneeToFootHelper, skipRotate=['x', 'z'], maintainOffset=True)

        noFlipHelper = self.makeNoFlipHelper(legHandle, self._noflipvector)
        kneePolePreT = pmc.listRelatives(self._rigControls['ik_knee'], p=True)
        pmc.parent(kneePolePreT, noFlipHelper)

        poleTwistHelper = pmc.group(kneePolePreT, name='hlp_ik_{0}_poletwist'.format(self._name))
        pmc.xform(poleTwistHelper, objectSpace=True, pivots=(0, 0, 0))
        pmc.connectAttr(kneeToFootHelper + '.rotateY', poleTwistHelper + '.rotateY')

        pmc.parent(noFlipHelper, kneeToFootHelper, legPreTransform, mainGroup)

        revIkVis = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))

        pmc.connectAttr(self._switchAttr, revIkVis + '.inputX')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_leg'] + '.visibility')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_leg'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_knee'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateX')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateY')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_toe'] + '.visibility')

        return mainGroup


class RiggingArm(Rigging):
    FK_CONTROL_ATTR_BASE = 'fkcontrol'
    IK_CONTROL_ATTR_BASE = 'ikcontrol'

    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, noFlipVector=None):
        super(RiggingArm, self).__init__(name, joints, parent, mainControl, switchboard)

        self._noflipvector = noFlipVector
        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(switchboard, self._name)
        self._fkjoints = self.makeJointSystems('fkj')
        self._ikjoints = self.makeJointSystems('ikj')
        self.connectJointChains(joints=self._joints, blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name))

        fkjointgrp = pmc.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = pmc.listRelatives(self._ikjoints[0], parent=True)[0]

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        pmc.parent(fkjointgrp, ikjointgrp, fkrig, ikrig, self.transform)

        pmc.hide((fkjointgrp, ikjointgrp))

        self.lockAndHide(True)

    def makeFkRig(self):
        mainGroup = pmc.group(empty=True, name='grp_fk_{0}_rig'.format(self._name))

        jnts = {'shoulder': self._fkjoints[0], 'elbow': self._fkjoints[1], 'wrist': self._fkjoints[2]}

        self._rigControls['fk_wrist'], wristPreTransform = makeControlNode(name='ctl_fk_{0}_wrist'.format(self._name),
                                                                           targetObject=jnts['wrist'])

        self._rigControls['fk_elbow'], elbowPreTransform = makeControlNode(name='ctl_fk_{0}_elbow'.format(self._name),
                                                                           targetObject=jnts['elbow'])

        self._rigControls['fk_shoulder'], shoulderPreTransform = \
            makeControlNode(name='ctl_fk_{0}_shoulder'.format(self._name), targetObject=jnts['shoulder'])

        self._rigControls['fk_gimbal_wrist'], armGimbalPreTransform = \
            makeControlNode(name='ctl_fk_{0}_wrist_gimbal'.format(self._name), targetObject=jnts['wrist'])

        parentTarget, worldTarget = self.makeOrientSwitchNodes(jnts['shoulder'], shoulderPreTransform,
                                                               name=self._name + '_fk')
        pmc.parent(parentTarget, worldTarget, mainGroup)

        pmc.orientConstraint(self._rigControls['fk_gimbal_wrist'], jnts['wrist'])
        pmc.orientConstraint(self._rigControls['fk_elbow'], jnts['elbow'])
        pmc.orientConstraint(self._rigControls['fk_shoulder'], jnts['shoulder'])

        pmc.parent(self._rigControls['fk_gimbal_wrist'], self._rigControls['fk_wrist'])
        pmc.parent(wristPreTransform, self._rigControls['fk_elbow'], absolute=True)
        pmc.parent(elbowPreTransform, self._rigControls['fk_shoulder'])

        pmc.parent(shoulderPreTransform, mainGroup)

        pmc.addAttr(self._rigControls['fk_wrist'], at='short', ln='showGimbal', min=0, max=1, defaultValue=1,
                     keyable=False, hidden=False)
        pmc.setAttr(self._rigControls['fk_wrist'] + '.showGimbal', edit=True, channelBox=True)

        if self._switchAttr:
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_shoulder'] + '.visibility')
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_elbow'] + '.visibility')
            pmc.connectAttr(self._switchAttr, self._rigControls['fk_wrist'] + '.visibility')

            gimbalVisMultNode = pmc.shadingNode('multiplyDivide', asUtility=True,
                                                 name='mul_fk_{0}_showGimbal'.format(self._name))

            pmc.connectAttr(self._switchAttr, gimbalVisMultNode + '.input1X')
            pmc.connectAttr(self._rigControls['fk_wrist'] + '.showGimbal',
                             gimbalVisMultNode + '.input2X')
            pmc.connectAttr(gimbalVisMultNode + '.outputX', self._rigControls['fk_gimbal_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_elbow'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_shoulder'] + '.visibility')

        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['fk_gimbal_wrist'] + '.visibility')

        pmc.delete(armGimbalPreTransform)

        return mainGroup

    def makeIkRig(self):
        mainGroup = pmc.group(empty=True, name='grp_ik_{0}_rig'.format(self._name))

        jnts = {'shoulder': self._ikjoints[0], 'elbow': self._ikjoints[1], 'wrist': self._ikjoints[2]}

        self._rigControls['ik_wrist'], armPreTransform = makeControlNode(name='ctl_ik_{0}'.format(self._name),
                                                                         targetObject=jnts['wrist'])

        self._rigControls['ik_gimbal_wrist'], armGimbalPreTransform = \
            makeControlNode(name='ctl_ik_{0}_wrist_gimbal'.format(self._name), targetObject=jnts['wrist'])

        handle = pmc.ikHandle(sj=jnts['shoulder'], ee=jnts['wrist'], sol='ikRPsolver',
                               n='ikh_{0}'.format(self._name))[0]

        self._rigControls['ik_elbow'], poleLine = makePoleVectorControlFromHandle('ctl_ik_{0}_pole'.format(self._name),
                                                                                  handle, parent=mainGroup)

        elbowPreT = pmc.listRelatives(self._rigControls['ik_elbow'], parent=True)[0]

        noFlipHelper = self.makeNoFlipHelper(handle, self._noflipvector)
        pmc.parent(elbowPreT, noFlipHelper)

        pmc.poleVectorConstraint(self._rigControls['ik_elbow'], handle)
        pmc.parent(self._rigControls['ik_gimbal_wrist'], self._rigControls['ik_wrist'])
        pmc.parent(handle, self._rigControls['ik_gimbal_wrist'])
        pmc.orientConstraint(self._rigControls['ik_gimbal_wrist'], jnts['wrist'])

        pmc.parent(armPreTransform, noFlipHelper, mainGroup)

        pmc.addAttr(self._rigControls['ik_wrist'], at='byte', ln='showGimbal', min=0, max=1,
                     defaultValue=1, keyable=False, hidden=False)
        pmc.setAttr(self._rigControls['ik_wrist'] + '.showGimbal', edit=True, channelBox=True)

        revIkVis = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))

        pmc.connectAttr(self._switchAttr, revIkVis + '.inputX')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_wrist'] + '.visibility')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_elbow'] + '.visibility')

        gimbalVisMultNode = pmc.shadingNode('multiplyDivide', asUtility=True,
                                             name='mul_ik_{0}_showGimbal'.format(self._name))

        pmc.connectAttr(revIkVis + '.outputX', gimbalVisMultNode + '.input1X')
        pmc.connectAttr(self._rigControls['ik_wrist'] + '.showGimbal', gimbalVisMultNode + '.input2X')
        pmc.connectAttr(gimbalVisMultNode + '.outputX', self._rigControls['ik_gimbal_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_wrist'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateX')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateY')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.rotateZ')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_elbow'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateX')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateY')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_gimbal_wrist'] + '.visibility')

        pmc.delete(armGimbalPreTransform)

        return mainGroup


class RiggingSpine(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, spline=None, switchboard=None):
        super(RiggingSpine, self).__init__(name, joints, parent, mainControl, switchboard)
        self._spline = spline

        getAttribute(self._switchboard, self._name + '_ikfk', min=0, max=1, defaultValue=0, keyable=True)

        self._switchAttr = '{0}.{1}_ikfk'.format(switchboard, self._name)

        # reparent fk joints to seperate pelvic rotation later
        self._fkjoints = self.makeJointSystems('fkj', makeConstraints=False)
        pmc.parent(self._fkjoints[0], self._fkjoints[1])

        self._ikjoints = self.makeJointSystems('ikj', makeConstraints=False)

        fkjointgrp = pmc.listRelatives(self._fkjoints[0], parent=True)[0]
        ikjointgrp = pmc.listRelatives(self._ikjoints[0], parent=True)[0]

        pmc.parent(fkjointgrp, ikjointgrp, self.transform)

        self.connectJointChains(joints=self._joints, blendAttr='{0}.{1}_ikfk'.format(switchboard, self._name),
                                stretchy=False, useConstraints=True)

        # Create, position, and set rotation order to controls
        self._rigControls['root'], rootCtlPreT = makeControlNode(name='ctl_{0}_root'.format(self._name),
                                                                 targetObject=self._joints[1], alignRotation=False)
        pmc.setAttr(self._rigControls['root'] + '.rotateOrder', ROO_XZY)
        self.lockAttrs.append(self._rigControls['root'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['root'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['root'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['root'] + '.visibility')

        fkrig = self.makeFkRig()
        ikrig = self.makeIkRig()

        pmc.parent(fkrig, self._rigControls['root'])
        pmc.parent(ikrig, self._rigControls['root'])
        pmc.parent(rootCtlPreT, self.transform)

        self.lockAndHide(True)

    def makeFkRig(self):
        mainGroup = pmc.group(empty=True, name='grp_fk_{0}_spinerig'.format(self._name))

        rootJoint = self._fkjoints[1]
        previousControl = None
        for jnt in self._fkjoints:
            if previousControl:
                control, preTransform = makeControlNode(name=jnt.replace('fkj_', 'ctl_fk_', 1), targetObject=jnt)

                if jnt == rootJoint:
                    pmc.parent(preTransform, mainGroup)
                    pmc.parentConstraint(control, jnt)
                else:
                    pmc.parent(preTransform, previousControl)
                    pmc.connectAttr(control + '.rotate', jnt + '.rotate')
            else:
                control, preTransform = makeControlNode(name=jnt.replace('fkj_', 'ctl_', 1), targetObject=rootJoint)

                pmc.parent(preTransform, mainGroup)
                pmc.parentConstraint(control, jnt, maintainOffset=True)

            pmc.connectAttr(self._switchAttr, control + '.visibility')

            self.lockAttrs.append(control + '.translateX')
            self.lockAttrs.append(control + '.translateY')
            self.lockAttrs.append(control + '.translateZ')
            self.lockAttrs.append(control + '.scaleX')
            self.lockAttrs.append(control + '.scaleY')
            self.lockAttrs.append(control + '.scaleZ')
            self.lockAttrs.append(control + '.visibility')

            self._rigControls[jnt] = control
            previousControl = control

        return mainGroup

    def makeIkRig(self):
        mainGroup = pmc.group(empty=True, name='grp_ik_{0}_spinerig'.format(self._name))

        if self._spline is None:
            splineIk = pmc.ikHandle(sj=self._ikjoints[0], ee=self._ikjoints[-1], sol='ikSplineSolver',
                                     parentCurve=False, createCurve=True, simplifyCurve=True, numSpans=2,
                                     rootOnCurve=False, n='sik_{0}_spine'.format(self._name))
            self._spline = pmc.rename(splineIk[2], 'spl_{0}_spinerig'.format(self._name))
        else:
            splineIk = pmc.ikHandle(sj=self._ikjoints[0], ee=self._ikjoints[-1], sol='ikSplineSolver',
                                     createCurve=False, rootOnCurve=False, curve=self._spline, parentCurve=False,
                                     n='sik_{0}_spine'.format(self._name))

        splineHandle = splineIk[0]
        pmc.setAttr(splineHandle + '.inheritsTransform', 0)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_lwr_spine'], spineCtl0PreT = makeControlNode(
            name='ctl_ik_{0}_lower_spine'.format(self._name),
            targetObject=self._ikjoints[1], alignRotation=False)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_mid_spine'], spineCtl1PreT = makeControlNode(
            name='ctl_ik_{0}_middle_spine'.format(self._name))

        midpoint = pmc.pointOnCurve('spl_spine', parameter=0.5, turnOnPercentage=True, position=True)
        pmc.xform(spineCtl1PreT, worldSpace=True, translation=midpoint)

        # Create, position, and set rotation order to controls
        self._rigControls['ik_upr_spine'], spineCtl2PreT = makeControlNode(
            name='ctl_ik_{0}_upper_spine'.format(self._name),
            targetObject=self._ikjoints[-1], alignRotation=False)

        pmc.setAttr(self._rigControls['ik_lwr_spine'] + '.rotateOrder', ROO_YXZ)
        pmc.setAttr(self._rigControls['ik_mid_spine'] + '.rotateOrder', ROO_YXZ)
        pmc.setAttr(self._rigControls['ik_upr_spine'] + '.rotateOrder', ROO_YXZ)

        pmc.parent(spineCtl2PreT, self._rigControls['ik_mid_spine'])
        pmc.parent(spineCtl0PreT, spineCtl1PreT, self._rigControls['root'])
        pmc.parentConstraint(self._rigControls['ik_lwr_spine'], self._ikjoints[0], maintainOffset=True)
        pmc.orientConstraint(self._rigControls['ik_upr_spine'], self._ikjoints[-1], maintainOffset=True)

        # connect controls to ik
        # Using self.joints in place of cluster nodes. They're simpler to work with and do the same thing
        clusterJoints = list()
        pmc.select(clear=True)
        clusterJoints.append(
            pmc.joint(position=pmc.xform(self._ikjoints[1], q=True, worldSpace=True, translation=True), radius=4,
                       name='clj_spine0'))
        pmc.select(clear=True)
        clusterJoints.append(
            pmc.joint(position=midpoint, radius=4, name='clj_spine1'))
        pmc.select(clear=True)
        clusterJoints.append(
            pmc.joint(position=pmc.xform(self._ikjoints[-1], q=True, worldSpace=True, translation=True), radius=4,
                       name='clj_spine2'))

        pmc.parent(clusterJoints[0], self._rigControls['ik_lwr_spine'])
        pmc.parent(clusterJoints[1], self._rigControls['ik_mid_spine'])
        pmc.parent(clusterJoints[2], self._rigControls['ik_upr_spine'])

        pmc.skinCluster(clusterJoints, self._spline, maximumInfluences=2)

        pmc.setAttr(splineHandle + '.dTwistControlEnable', 1)

        pmc.setAttr(splineHandle + '.dWorldUpType', 4)
        pmc.setAttr(splineHandle + '.dWorldUpAxis', 0)

        pmc.setAttr(splineHandle + '.dWorldUpVectorX', 0.0)
        pmc.setAttr(splineHandle + '.dWorldUpVectorY', 0.0)
        pmc.setAttr(splineHandle + '.dWorldUpVectorZ', -1.0)
        pmc.setAttr(splineHandle + '.dWorldUpVectorEndX', 0.0)
        pmc.setAttr(splineHandle + '.dWorldUpVectorEndY', 0.0)
        pmc.setAttr(splineHandle + '.dWorldUpVectorEndZ', -1.0)

        pmc.connectAttr(self._rigControls['ik_lwr_spine'] + '.worldMatrix[0]',
                         splineHandle + '.dWorldUpMatrix')

        pmc.connectAttr(self._rigControls['ik_upr_spine'] + '.worldMatrix[0]',
                         splineHandle + '.dWorldUpMatrixEnd')

        pmc.parent(splineHandle, self._spline, mainGroup)

        revIkVis = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_ik_visibility'.format(self._name))
        pmc.connectAttr(self._switchAttr, revIkVis + '.inputX')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_lwr_spine'] + '.visibility')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_mid_spine'] + '.visibility')
        pmc.connectAttr(revIkVis + '.outputX', self._rigControls['ik_upr_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_lwr_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_mid_spine'] + '.visibility')

        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['ik_upr_spine'] + '.visibility')

        return mainGroup


class RiggingFingers(Rigging):
    FINGER_CURL_ATTR_NAME = 'curl'
    FINGER_STRETCH_ATTR_NAME = 'stretch'
    FINGER_VIS_ATTR_NAME = 'extraControls'

    def __init__(self, name, joints, parent=None, mainControl=None, minStretch=-1.5, maxStretch=1.5,
                 reverseStretch=False):
        super(RiggingFingers, self).__init__(name, joints, parent, mainControl)

        self._minStretch = minStretch
        self._maxStretch = maxStretch
        self._reverseStretch = reverseStretch

        handGrp = self.makeRig()

        pmc.parent(handGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        rootTransforms = list()

        for root in self._joints:
            fingers = [root]
            childFingers = pmc.listRelatives(root, children=True, allDescendents=True, type='joint')
            childFingers.reverse()
            fingers.extend(childFingers)

            curlAttr = None
            stretchAttr = None
            visibilityAttr = None

            previousControl = None
            for fng in fingers[:-1]:
                control, preTransform = makeControlNode(name=fng.replace('rig_', 'ctl_', 1), targetObject=fng)

                if previousControl:
                    pmc.parent(preTransform, previousControl)
                    drivenGrp = pmc.group(control, name='hlp_' + control)

                    curlNode = pmc.shadingNode('multiplyDivide', asUtility=True,
                                                n='mul_{0}_{1}_curl'.format(self._name, fng))

                    pmc.setAttr(curlNode + '.input2Z', -11.0)
                    pmc.connectAttr(curlAttr, curlNode + '.input1Z')
                    pmc.connectAttr(curlNode + '.outputZ', drivenGrp + '.rotateZ')

                    stretchRangeNode = pmc.shadingNode('setRange', asUtility=True,
                                                        n='rng_{0}_{1}_stretch'.format(self._name, fng))

                    if self._reverseStretch:
                        revStretchNode = pmc.shadingNode('reverse', asUtility=True,
                                                          n='rev_{0}_{1}_stretch'.format(self._name, fng))
                        pmc.connectAttr(stretchAttr, revStretchNode + '.inputX')
                        pmc.connectAttr(revStretchNode + '.outputX', stretchRangeNode + '.valueX')
                    else:
                        pmc.connectAttr(stretchAttr, stretchRangeNode + '.valueX')

                    pmc.setAttr(stretchRangeNode + '.minX', self._minStretch)
                    pmc.setAttr(stretchRangeNode + '.maxX', self._maxStretch)
                    pmc.setAttr(stretchRangeNode + '.oldMinX', -10.0)
                    pmc.setAttr(stretchRangeNode + '.oldMaxX', 10.0)

                    pmc.connectAttr(stretchRangeNode + '.outValueX', drivenGrp + '.translateX')

                    pmc.connectAttr(visibilityAttr, preTransform + '.visibility')
                else:
                    curlAttr = getAttribute(control, self.FINGER_CURL_ATTR_NAME,
                                            min=-10.0, max=10.0, defaultValue=0, keyable=True)
                    stretchAttr = getAttribute(control, self.FINGER_STRETCH_ATTR_NAME,
                                               min=-10.0, max=10.0, defaultValue=0, keyable=True)
                    visibilityAttr = getAttribute(control, self.FINGER_VIS_ATTR_NAME, at='short',
                                                  min=0, max=1, defaultValue=1, keyable=True, hidden=False)

                    pmc.setAttr(visibilityAttr, edit=True, channelBox=True)

                pmc.parentConstraint(control, fng)

                if fng != root:
                    self.lockAttrs.append(control + '.rotateX')
                    self.lockAttrs.append(control + '.rotateY')
                else:
                    rootTransforms.append(preTransform)

                self.lockAttrs.append(control + '.translateX')
                self.lockAttrs.append(control + '.translateY')
                self.lockAttrs.append(control + '.translateZ')
                self.lockAttrs.append(control + '.scaleX')
                self.lockAttrs.append(control + '.scaleY')
                self.lockAttrs.append(control + '.scaleZ')
                self.lockAttrs.append(control + '.visibility')

                previousControl = control
                self._rigControls[fng] = control

        groupNode = pmc.group(empty=True, name='grp_{0}_fingers'.format(self._name))
        pmc.parentConstraint(self._parent, groupNode, maintainOffset=False)
        pmc.parent(rootTransforms, groupNode)

        return groupNode


class RiggingHead(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        super(RiggingHead, self).__init__(name, joints, parent, mainControl, switchboard)

        handGrp = self.makeRig()

        pmc.parent(handGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        mainGroup = pmc.group(empty=True, name='grp_{0}_headrig'.format(self._name))

        jnts = {'neck': self._joints[0], 'head': self._joints[1]}

        self._rigControls['neck'], preTransform = makeControlNode(name='ctl_{0}_neck'.format(self._name),
                                                                  targetObject=jnts['neck'])
        self._rigControls['head'], preHeadTransform = makeControlNode(name='ctl_{0}_head'.format(self._name),
                                                                      targetObject=jnts['head'])

        parentTarget = pmc.group(empty=True, name='tgt_{0}_local'.format(self._name))
        worldTarget = pmc.group(empty=True, name='tgt_{0}_world'.format(self._name))
        alignObjects([parentTarget, worldTarget], preTransform)

        pmc.parentConstraint(self._parent, parentTarget, maintainOffset=True)
        pmc.parentConstraint(self._mainControl, worldTarget, maintainOffset=True)

        pmc.pointConstraint(jnts['neck'], preTransform, maintainOffset=False)

        constraint = pmc.orientConstraint(parentTarget, preTransform,
                                           n=self._name + '_orientConstraint')[0]
        pmc.orientConstraint(worldTarget, preTransform, n=self._name + '_orientConstraint')
        pmc.setAttr(constraint + '.interpType', 2)  # shortest interpolation (less flipping)

        orientAttr = getAttribute(self._switchboard, self._name + '_isolation',
                                  min=0, max=1, defaultValue=0, keyable=True)

        revAttrNode = pmc.shadingNode('reverse', asUtility=True, name='rev_{0}_isolation'.format(self._name))

        orientWeightList = pmc.orientConstraint(constraint, q=True, weightAliasList=True)
        pmc.connectAttr(orientAttr, orientWeightList[1])
        pmc.connectAttr(orientAttr, revAttrNode + '.inputX')
        pmc.connectAttr(revAttrNode + '.outputX', orientWeightList[0])

        pmc.parent([parentTarget, worldTarget], mainGroup)

        pmc.parent(preHeadTransform, self._rigControls['neck'])
        pmc.orientConstraint(self._rigControls['neck'], jnts['neck'])
        pmc.connectAttr(self._rigControls['head'] + '.rotate', jnts['head'] + '.rotate')

        self.lockAttrs.append(self._rigControls['neck'] + '.translateX')
        self.lockAttrs.append(self._rigControls['neck'] + '.translateY')
        self.lockAttrs.append(self._rigControls['neck'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['neck'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['neck'] + '.visibility')

        self.lockAttrs.append(self._rigControls['head'] + '.translateX')
        self.lockAttrs.append(self._rigControls['head'] + '.translateY')
        self.lockAttrs.append(self._rigControls['head'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['head'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['head'] + '.visibility')

        pmc.parent(preTransform, mainGroup)

        return mainGroup


class RiggingClavicle(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None):
        super(RiggingClavicle, self).__init__(name, joints, parent, mainControl, switchboard)

        clavGrp = self.makeRig()

        pmc.parent(clavGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        jnts = {'clav': self._joints[0]}

        self._rigControls['clav'], preTransform = makeControlNode(name='ctl_{0}'.format(self._name),
                                                                  targetObject=jnts['clav'])

        pmc.connectAttr(self._rigControls['clav'] + '.rotate', jnts['clav'] + '.rotate')
        pmc.parentConstraint(self._parent, preTransform, maintainOffset=True)

        self.lockAttrs.append(self._rigControls['clav'] + '.translateX')
        self.lockAttrs.append(self._rigControls['clav'] + '.translateY')
        self.lockAttrs.append(self._rigControls['clav'] + '.translateZ')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleX')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleY')
        self.lockAttrs.append(self._rigControls['clav'] + '.scaleZ')
        self.lockAttrs.append(self._rigControls['clav'] + '.visibility')

        return preTransform


class RiggingGenericFK(Rigging):
    def __init__(self, name, joints, parent=None, mainControl=None, switchboard=None, isolation=False):
        super(RiggingGenericFK, self).__init__(name, joints, parent, mainControl, switchboard)
        self._isolation = isolation
        fkGrp = self.makeRig()

        pmc.parent(fkGrp, self.transform)

        self.lockAndHide(True)

    def makeRig(self):
        allPreTransforms = list()
        for i, jnt in enumerate(self._joints):
            self._rigControls[i], preTransform = makeControlNode(name='ctl_{0}'.format(self._name), targetObject=jnt)

            pmc.connectAttr(self._rigControls[i] + '.rotate', jnt + '.rotate')

            self.lockAttrs.append(self._rigControls[i] + '.translateX')
            self.lockAttrs.append(self._rigControls[i] + '.translateY')
            self.lockAttrs.append(self._rigControls[i] + '.translateZ')
            self.lockAttrs.append(self._rigControls[i] + '.scaleX')
            self.lockAttrs.append(self._rigControls[i] + '.scaleY')
            self.lockAttrs.append(self._rigControls[i] + '.scaleZ')
            self.lockAttrs.append(self._rigControls[i] + '.visibility')

            allPreTransforms.append(preTransform)

        mainGroup = allPreTransforms[0]
        if self._switchboard and self._isolation:
            targetNodes = self.makeOrientSwitchNodes(self._joints[0], mainGroup)
            pmc.parent(targetNodes, mainGroup)
        else:
            pmc.parentConstraint(self._parent, mainGroup, maintainOffset=True)

        return mainGroup


        # CODE SNIPPETS USED TO CREATE GOLDIE & DANIEL

        # import cogbiped
        # reload(cogbiped)

        # from cogbiped import *

        # joints = [u'rig_left_leg_hip', u'rig_left_leg_knee', u'rig_left_leg_ankle', u'rig_left_leg_ball', u'rig_left_leg_toe']
        # RiggingLeg(name='left_leg', parent='rig_spine0', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=[0, -1, 0])

        # joints = [u'rig_right_leg_hip', u'rig_right_leg_knee', u'rig_right_leg_ankle', u'rig_right_leg_ball',
        # u'rig_right_leg_toe']
        # RiggingLeg(name='right_leg', parent='rig_spine0', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=[0, -1, 0])

        # joints = [u'rig_spine0', u'rig_spine1', u'rig_spine2', u'rig_spine3', u'rig_spine4']
        # RiggingSpine(name='spine', parent='ctl_main', joints=joints, mainControl='ctl_main',
        # spline='spl_spine', switchboard='ctl_settings')

        # joints = [u'rig_left_arm_shoulder', u'rig_left_arm_elbow', u'rig_left_arm_wrist']
        # RiggingArm(name='left_arm', parent='rig_left_arm_clavicle', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=(1, 0, 0))

        # joints = [u'rig_right_arm_shoulder', u'rig_right_arm_elbow', u'rig_right_arm_wrist']
        # RiggingArm(name='right_arm', parent='rig_right_arm_clavicle', joints=joints, mainControl='ctl_main',
        # switchboard='ctl_settings', noFlipVector=(-1, 0, 0))

        # joints = [u'rig_spine5', u'rig_head']
        # RiggingHead(name='head', parent='rig_spine4', joints=joints, mainControl='ctl_main',
        #             switchboard='ctl_settings')

        # joints = [u'rig_left_fng_thumb0', u'rig_left_fng_index0', u'rig_left_fng_middle0', u'rig_left_fng_pinky0']
        # RiggingFingers(name='left_hand', parent='rig_left_arm_wrist', joints=joints, mainControl='ctl_main',
        #                minStretch=-0.1, maxStretch=0.1)

        # joints = [u'rig_right_fng_thumb0', u'rig_right_fng_index0', u'rig_right_fng_middle0', u'rig_right_fng_pinky0']
        # RiggingFingers(name='right_hand', parent='rig_right_arm_wrist', joints=joints, mainControl='ctl_main',
        #                minStretch=-0.1, maxStretch=0.1, reverseStretch=True)

        # joints = [u'rig_left_arm_clavicle']
        # RiggingClavicle(name='left_clav', parent='rig_spine4', joints=joints, mainControl='ctl_main')

        # joints = [u'rig_right_arm_clavicle']
        # RiggingClavicle(name='right_clav', parent='rig_spine4', joints=joints, mainControl='ctl_main')