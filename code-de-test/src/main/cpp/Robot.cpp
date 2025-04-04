//--------------------------------------------------------
// Introduction des library
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <CANVenom.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cameraserver/CameraServer.h>
#include <frc/Servo.h>

//-------------------------------------------------------
// Base du code
class Robot : public frc::TimedRobot {
 public:
  Robot() {

    //---------------------------------------------------
    // Intégrer les deux moteurs de controle au meme parent
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_left);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &CANVenom_right);
    
    //---------------------------------------------------
    // Selection du moteur qui doit être inverser pour
    // assurer le bon controle du robot
    CANVenom_right.SetInverted(true);

    //---------------------------------------------------
    // Sécuriter des moteurs
    m_robotDrive.SetExpiration(100_ms);

    //---------------------------------------------------
    // Start un timer (je pense)
    m_timer.Start();

    
    //---------------------------------------------------
    // Introduction du system pour la caméra
    std::thread visionThread(VisionThread);
    visionThread.detach();
    
    //--------------------------------------------------
    
  }

//----------------------------------------------------
// Utile pour des automatisations
  void AutonomousInit() override { 
    m_timer.Reset(); 
    m_timer.Start();
    state = 0;

  } // Initialisation du code pour un auto

  void AutonomousPeriodic() override {
    

    //m_robotDrive.ArcadeDrive(pr_speed*speed, pr_rotation*turn);
  } //  Une section de code pour lire les variables périodiquement
//----------------------------------------------------

//----------------------------------------------------
// Utile pour le Teleop
  void TeleopInit() override {} // Initialisation du code pour le Teleop uniquement un seul fois

  void TeleopPeriodic() override { //  Une section de code pour lire les variables périodiquement
    
    // Défnition des variables de controle
    double pr_speed = 0;
    double pr_rotation = 0;
    double forward = 0;
    double backward = 0;
    
    double speed = 0;
    if (m_controller.GetYButton()){
      if (joystick_mode == true) {
        joystick_mode = false;
      } else {
        joystick_mode = true;
      }
    }
    if (joystick_mode == true) {
      speed = -1*m_controller.GetRightY();
    } else {
      forward = m_controller.GetRightTriggerAxis();
      backward = m_controller.GetLeftTriggerAxis();
      speed = forward - backward;
    }
    double turn = m_controller.GetLeftX();
    // Definition du code de controle
    

    // Définition de la variable speed pour avancerspeed = forward - backward;

    // Definition du code  pour l'action du robot ( avancer, reculer et touner)
    

    if (m_controller.GetAButton()){
      Slow = true;
    } else if (m_controller.GetBButton()){
      Slow = false;
    }
    
    if (Slow == true ){
      pr_speed = -0.7;
      pr_rotation = 0.6;
    }
    
    if (Slow == false){
      pr_speed = -0.9;
      pr_rotation = 0.8;
    }
    m_robotDrive.ArcadeDrive(pr_speed*speed, pr_rotation*turn);
    //Définition du code pour faire la rotion de la caméra si la vitesse du robot deviens négative
    if (m_controller.GetPOV() == 270){
      servo_cam.SetAngle(65.67);
    } else if (m_controller.GetPOV() == 180){
      servo_cam.SetAngle(137);
    } else if (m_controller.GetPOV() == 90){
      servo_cam.SetAngle(212.67);
    } else if (m_controller.GetPOV() == 0) {
      servo_cam.SetAngle(0);
    } else if (speed > -0.1){
      servo_cam.SetAngle(0);
    } else {
      servo_cam.SetAngle(137);
    }

    // Reglage vitesse
    double pov;
    pov = m_controller.GetPOV();

    if (pov == 90 && pr_speed < 1)
    {
      pr_speed += 0.01;
      
    }
    if (pov == 270 && pr_speed > 0.1)
    {
      pr_speed -= 0.01;
    }
    
  }

//---------------------------------------------------
// Partie du code pour les test

//---------------------------------------------------

//--------------------------------------------------
// Section des variable
 private:

  //Définition des moteurs Venom
  pwf::CANVenom CANVenom_left{2};
  pwf::CANVenom CANVenom_right{1};

  //Définition des servo moteurs
  frc::Servo servo_cam{1};
  
  //Définition de comment les moteurs doivent se comporter
  frc::DifferentialDrive m_robotDrive{
  CANVenom_left, CANVenom_right
  };

  //Définition de quelle manette est utiliser par la variable m_controller
  frc::XboxController m_controller{0};

  //Définition Timer
  frc::Timer m_timer;

  int state = 0;

  bool Slow = false;
  bool joystick_mode = false;
  //Définition de la fonction VisionThread qui est utiliser pour l'utilisation du camera sur le robot pour avoir un retour
  static void VisionThread() {
      
      //Définis que la variable camera est maintenant utiliser
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

      //Définis que la camera a une définition de 640x480
      camera.SetResolution(160, 120);

      //Définis que la camera a une compressention de 
      //camera.SetPixelFormat();

      //Définis le nbre fps de la camera
      camera.SetFPS(30);

      // Met en action un serveur cvSink et indique a la camera de capturer la vidéo
      cs::CvSink cvSink = frc::CameraServer::GetVideo();

      // Met en place le service de renvois de video au dashboard avec un rectangle dessiner
      cs::CvSource outputStream =
          frc::CameraServer::PutVideo("Gray  Video", 1080, 720);

      // Défini un claque statique pour un rectangle
      cv::Mat mat;
      cv::Mat grayMats;
      while (true) {
        // si jamais il y a un probleme dans la capture de l'image
        // depuis la camera, renvoie un code d'erreur
        if (cvSink.GrabFrame(mat) == 0) {

          // Envoie l'erreur dans les logs
          outputStream.NotifyError(cvSink.GetError());

          // Continue
          continue;

        }
        cv::cvtColor(mat, grayMats, cv::COLOR_BGR2GRAY);


        // Place un rectangle dans la vidéo
        

        // Donne au dashboard une nouvelle image comme un nouveau calque
        outputStream.PutFrame(grayMats);

      }
    }
    
};

// Donne l'indiquation de partir le robot
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif