import pinocchio 
import crocoddyl
import mujoco
import mujoco_viewer
import numpy as np

# define mujoco dynamic derivatives fx and fu
def derivative_dynamics(model,data,eps):
            A=np.zeros((2*model.nv, 2*model.nv))
            B=np.zeros((2*model.nv, model.nu))
            centered=True
            mujoco.mjd_transitionFD(model,data,eps,centered,A,B,None,None)
            return A,B

# build integration action model for crocoddyl 
class MujocoActionModel(crocoddyl.differentialActionModelAbstract):
    def __init__(self,state,costModel,xml_path):
        crocoddyl.DifferentialActionModelAbstract.__init__(self,state,state.nv,costModel.nr)
        self.cost=costModel
        self.enable_force=True
        self.armature=np.zeros(0)
        self.MjModel=mujoco.MjModel.from_xml_path(xml_path)
        self.MjData=mujoco.MjData(self.MjModel)
        def derivative_dynamics(model,data,eps):
            A=np.zeros((2*model.nv, 2*model.nv))
            B=np.zeros((2*model.nv, model.nu))
            centered=True
            mujoco.mjd_transitionFD(model,data,eps,centered,A,B,None,None)
            return A,B

    def calc(self, data, x, u=None):
        if u is None:
            u=self.unone
            q, v=x[:self.state.nq],x[-self.state.nv:]
            self.MjData.qpos, self.MjData.qvel=q, v
            self.MjData.ctrl=u
        data.xout=mujoco.mj_step(self.MjModel,self.MjData)
        pinocchio.forwardKinematics(self.state.pinocchio, data.pinocchio, q, v)
        self.costs.calc(data.costs,x,u)
        data.cost=data.costs.cost

    def calcDiff(self,data, x, u=None):
        if u is None:
            u=self.unone
        if True:
            self.calc(data,x,u)
        A,B=derivative_dynamics(self.MjModel,self.MjData,1e-6)
        data.Fx=A
        data.Fu=B
    
    def createData(self):
        data = crocoddyl.DifferentialActionModelAbstract.createData(self)
        data.pinocchio = pinocchio.Data(self.state.pinocchio)
        data.multibody = crocoddyl.DataCollectorMultibody(data.pinocchio)
        data.costs = self.costs.createData(data.multibody)
        data.costs.shareMemory(data) # this allows us to share the memory of cost-terms of action model
        return data

urdf_path="/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.urdf"
robot_model=pinocchio.buildModelFromUrdf(urdf_path)
target = np.array([1,1,1])
state = crocoddyl.StateMultibody(robot_model)
frameTranslationalResidual=crocoddyl.ResidualModelFrameTranslation(state,robot_model.getFrameId(""),target)
goalTrackingCost= crocoddyl.CostModelResidual(state,frameTranslationalResidual)
xRegCost=crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelState(state))
uRegCost=crocoddyl.CostModelResidual(state, crocoddyl.ResidualModelControl(state))

runningCostModel=crocoddyl.CostModelSum(state)
terminalCostModel=crocoddyl.CostModelSum(state)

runningCostModel.addCost("gripperPose", goalTrackingCost, 1e2)
runningCostModel.addCost("stateReg", xRegCost, 1e-4)
runningCostModel.addCost("ctrlReg", uRegCost, 1e-7)
terminalCostModel.addCost("gripperPose", goalTrackingCost, 1e5)
terminalCostModel.addCost("stateReg", xRegCost, 1e-4)
terminalCostModel.addCost("ctrlReg", uRegCost, 1e-7)

DT










        

