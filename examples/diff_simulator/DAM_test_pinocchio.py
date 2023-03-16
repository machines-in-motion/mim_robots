import crocoddyl
import pinocchio
import example_robot_data
import numpy as np 

talos_arm=example_robot_data.load('talos_arm')
robot_model=talos_arm.model
robot_model=pinocchio.buildModelFromUrdf("/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.urdf")


# Defining a initial state
q0 = np.array([0.173046, 1., -0.52366, 0., 0., 0.1, -0.005])
x0 = np.concatenate([q0, np.zeros(talos_arm.model.nv)])


class DifferentialFwdDynamics(crocoddyl.DifferentialActionModelAbstract):
    def __init__(self, state, costModel):
        crocoddyl.DifferentialActionModelAbstract.__init__(self, state, state.nv, costModel.nr)
        self.costs = costModel
        self.enable_force = True
        

    def calc(self, data, x, u=None):
        if u is None:
            u = self.unone
        q, v = x[:self.state.nq], x[-self.state.nv:]
        # Computing the dynamics using ABA or manually for armature case
        if self.enable_force:
            data.xout = pinocchio.aba(self.state.pinocchio, data.pinocchio, q, v, u)
        else:
            pinocchio.computeAllTerms(self.state.pinocchio, data.pinocchio, q, v)
            data.M = data.pinocchio.M
            if self.armature.size == self.state.nv:
                data.M[range(self.state.nv), range(self.state.nv)] += self.armature
            data.Minv = np.linalg.inv(data.M)
            data.xout = data.Minv * (u - data.pinocchio.nle)
        # Computing the cost value and residuals
        pinocchio.forwardKinematics(self.state.pinocchio, data.pinocchio, q, v)
        pinocchio.updateFramePlacements(self.state.pinocchio, data.pinocchio)
        self.costs.calc(data.costs, x, u)
        data.cost = data.costs.cost

    def calcDiff(self, data, x, u=None):
        q, v = x[:self.state.nq], x[-self.state.nv:]
        if u is None:
            u = self.unone
        if True:
            self.calc(data, x, u)
        # Computing the dynamics derivatives
        if self.enable_force:
            pinocchio.computeABADerivatives(self.state.pinocchio, data.pinocchio, q, v, u)
            data.Fx = np.hstack([data.pinocchio.ddq_dq, data.pinocchio.ddq_dv])
            data.Fu = data.pinocchio.Minv
        else:
            pinocchio.computeRNEADerivatives(self.state.pinocchio, data.pinocchio, q, v, data.xout)
            data.Fx = -np.hstack([data.Minv * data.pinocchio.dtau_dq, data.Minv * data.pinocchio.dtau_dv])
            data.Fu = data.Minv
        # Computing the cost derivatives
        self.costs.calcDiff(data.costs, x, u)

    def set_armature(self, armature):
        if armature.size is not self.state.nv:
            print('The armature dimension is wrong, we cannot set it.')
        else:
            self.enable_force = False
            self.armature = armature.T

    def createData(self):
        data = crocoddyl.DifferentialActionModelAbstract.createData(self)
        data.pinocchio = pinocchio.Data(self.state.pinocchio)
        data.multibody = crocoddyl.DataCollectorMultibody(data.pinocchio)
        data.costs = self.costs.createData(data.multibody)
        data.costs.shareMemory(data) # this allows us to share the memory of cost-terms of action model
        return data
    


target = np.array([0,0,1])
state = crocoddyl.StateMultibody(robot_model)
frameTranslationResidual = crocoddyl.ResidualModelFrameTranslation(state,
                                            robot_model.getFrameId("L7"),
                                            target)
goalTrackingCost = crocoddyl.CostModelResidual(state, frameTranslationResidual)
xRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelState(state))
uRegCost = crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelControl(state))

# Create cost model per each action model
runningCostModel = crocoddyl.CostModelSum(state)
terminalCostModel = crocoddyl.CostModelSum(state)

# Then let's added the running and terminal cost functions
runningCostModel.addCost("gripperPose", goalTrackingCost, 1e2)
runningCostModel.addCost("stateReg", xRegCost, 1e-4)
runningCostModel.addCost("ctrlReg", uRegCost, 1e-7)
terminalCostModel.addCost("gripperPose", goalTrackingCost, 1e5)
terminalCostModel.addCost("stateReg", xRegCost, 1e-4)
terminalCostModel.addCost("ctrlReg", uRegCost, 1e-7)

# Running and terminal action models
DT = 1e-3
actuationModel = crocoddyl.ActuationModelFull(state)
runningModel = crocoddyl.IntegratedActionModelEuler(
    DifferentialFwdDynamics(state,  runningCostModel), DT)
terminalModel = crocoddyl.IntegratedActionModelEuler(
    DifferentialFwdDynamics(state, terminalCostModel), 0.)

T = 250
problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)

ddp=crocoddyl.SolverDDP(problem)
log=crocoddyl.CallbackLogger()

display = crocoddyl.MeshcatDisplay(talos_arm,4,4,False)
ddp.setCallbacks([log,crocoddyl.CallbackVerbose(),crocoddyl.CallbackDisplay(display)])

ddp.solve()

xs = ddp.xs
us = ddp.us
print(xs[0],us[0])

# Printing the reached position
frame_idx = robot_model.getFrameId("L7")
xT = ddp.xs[-1]
qT = xT[:robot_model.nq]
vT =np.zeros_like(qT)

robot_data=robot_model.createData()
viz=pin.visualize.MeshcatVisualizer(robot.model,)
print ("The reached pose by the wrist is")
pinocchio.forwardKinematics(robot_model,robot_data,qT,vT,vT)
pinocchio.updateFramePlacements(robot_model,robot_data)
print (robot_data.oMf[robot_model.getFrameId("L7")])

#%%
import robot_loader
# %%
import robot_envs.robot_loader
# %%
d