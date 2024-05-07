#pragma once
#ifndef INCLUDED_IMSL_DIFFDRIVECONTROLLERTASKS_H
#define INCLUDED_IMSL_DIFFDRIVECONTROLLERTASKS_H

#include "ControllerTasks.h"
#include "DiffDriveControl.h"

// namespace imsl::vehiclecontrol { // requires C++17
/* nested namespace definition: namespace A::B::C { ... } is equivalent to 
   namespace A { namespace B { namespace C { ... } } }. */
namespace imsl { 
  namespace vehiclecontrol {

  template<typename real_t>
  class DiffDriveControllerTasks : public ControllerTasks<real_t> {
    private:
    using CT = ControllerTasks<real_t>;

    DifferentialDriveController<real_t> Controller;
    RT_Mutex ContrMutex;        // Mutex for controller object

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigenlib 16 Byte alignement
    
    using WheelVel = Eigen::Matrix<real_t, 2, 1>;
    using VehicleVel = Eigen::Matrix<real_t, 2, 1>;

    protected:

    int PoseControlInterface(PoseV_t reference, PoseV_t actual, vPose<real_t> &RKSGeschw, real_t* vWheel_ref) override {
      VehicleVel RFVel;      // vehicle velocity in robot frame
      WheelVel WheelRefVel;  // reference velocities of the wheels 

      try {
        // call pose control method, obtain velocity in robot frame 
        RFVel = Controller.poseControl(reference, actual);
      }
      catch (mrc_stat e) { // Schleppfehler aufgetreten
//        hal_amplifiers_disable();
        CT::SetControllerMode(CtrlMode::OFF);

        CT::SetControllerErrorStatus(MRC_LAGEERR);
        log_message(log_error, "Schleppfehler Lageregler");
      }

      // convert to global native type vPose<real_t>
      // 2 degrees of freedom in robot frame: vx, omega only
      RKSGeschw.vx = RFVel(0);
      RKSGeschw.vy = 0.0;
      RKSGeschw.omega = RFVel(1);

      // calculate reference velocities of the wheels 
      WheelRefVel = Controller.vRF2vWheel(RFVel);

      // convert to global native type real_t*
      for(int i = 0; i < CT::NumbWheels; i++) {
        vWheel_ref[i] = WheelRefVel(i);
      }
      
      return 0;
    } // end of method PoseControlInterface
    
    void WheelControlInterface(real_t* ReferenceVel, real_t* ActualVel, real_t* CorrectingVel) override {
      WheelVel RefVel = WheelVel::Zero(),
               ActVel = WheelVel::Zero(),
               CorrVel = WheelVel::Zero();
      for(int i = 0; i < CT::NumbWheels; i++) {
        RefVel(i) = ReferenceVel[i];
        ActVel(i) = ActualVel[i];
      }
        
      // call wheel control algorithm
      try {
        CorrVel = Controller.wheelControl(RefVel, ActVel);

        // convert correcting variable to native type real_t*
        for(int i = 0; i < CT::NumbWheels; i++) {
            CorrectingVel[i] = CorrVel(i);
        }
      } // end of try
      catch (mrc_stat e) {
//        hal_amplifiers_disable();
        CT::SetControllerMode(CtrlMode::OFF);
    
        CT::SetControllerErrorStatus(MRC_DRZERR);  
        log_message(log_error, "Schleppfehler Drehzahlregler");
      }
    } // end of method WheelControlInterface
    

    void OdometryInterface(real_t* RadDeltaPhi, dPose<real_t> &VehicleMovement) override {
      WheelVel wm;
      VehicleVel vm;
        
      for (int i = 0; i < CT::NumbWheels; i++) {
          wm(i) = RadDeltaPhi[i];
      }
    
      // movements in robot frame
      vm = Controller.vWheel2vRF(wm);

      // convert vm to global type dPose<real_t>
      VehicleMovement.x = vm(0); 
      VehicleMovement.y = 0.0; 
      VehicleMovement.theta = vm(1);
      
      Pose<real_t> oldPose, newPose;
      CT::getPose(oldPose);
    
      ContrMutex.lock();
      newPose = Controller.odometry(oldPose, wm);
      ContrMutex.unlock();
      
      CT::setPose(newPose);

     } // end of method OdometryInterface
    
    void ManualModeInterface(vPose<real_t> &vRFref, vPose<real_t> &vRFold, Pose<real_t> &ManuPose) override {
      dPose<real_t> PoseErrorWF, PoseErrorRF;
      Pose<real_t> aktPose;
      vPose<real_t> vWFref;

      vRFref = Controller.velocityFilter(vRFref, vRFold); 

      CT::getPose(aktPose);     // obtain actual pose

      /* V von RKS -> WKS transformieren */
      vWFref = vRF2vWF<real_t>(vRFref, aktPose.theta); 

      /* Positionssollwert anpassen, Geschwindigkeit WKS einstellen */
      ManuPose.x += vWFref.vx * CT::Ta.FzLage;
      ManuPose.y += vWFref.vy * CT::Ta.FzLage;
      ManuPose.theta += vWFref.omega * CT::Ta.FzLage;

      // spezifisch fuer Differentialantrieb 

      // Regelabweichung im RKS bestimmen und in y auf Null setzen
      PoseErrorWF.x = ManuPose.x - aktPose.x;
      PoseErrorWF.y = ManuPose.y - aktPose.y;
      PoseErrorWF.theta = 0.0; // dont't care

      // tranformation of pose error into robot frame
      PoseErrorRF = dWF2dRF<real_t>(PoseErrorWF, aktPose.theta);
      // Differentialantrieb: Regelabweichung in der Y-Komponente auf 0 setzen
      PoseErrorRF.y = 0.0;
      PoseErrorWF = dRF2dWF<real_t>(PoseErrorRF, aktPose.theta);

      ManuPose.x = aktPose.x + PoseErrorWF.x;
      ManuPose.y = aktPose.y + PoseErrorWF.y;

    } // end of method ManualModeInterface

    int Init(Fahrzeug_t * fz, ReglerParam_t Regler, Abtastzeit_t Ta) override {
      CT::NumbWheels = 2;
      CT::DegFreed = 2;
      Controller.Init(fz, Regler, Ta); 

      if (!ContrMutex.create()) {
        log_message(log_error, "%s: Can init mutex: %s", __FUNCTION__,
        strerror(errno));
        return -1;
      }

      return CT::Init(fz, Regler, Ta); // call base class Init
    } // end of method Init
/*
    int CleanUp() override {
      ContrMutex.Delete();
      return CT::CleanUp(); // call base class CleanUp
    } // end of method CleanUp
*/    
    void PoseUpdate(dPose<real_t> PoseDelta, unsigned int Divisor) override {
      ContrMutex.lock();
      Controller.poseUpdate(PoseDelta, Divisor); 
      ContrMutex.unlock();
    } // end of method PoseUpdate
    
    void HeadingUpdate(real_t HeadingDelta, unsigned int Divisor) override {
      ContrMutex.lock();
      Controller.headingUpdate(HeadingDelta, Divisor); 
      ContrMutex.unlock();
    } // end of method HeadingUpdate

  }; // end of class DiffDriveControllerTasks

} } // end of namespace imsl::vehiclecontrol

#endif // INCLUDED__H
