// File:          Robot_Controller_z5207471.cpp
// Date:
// Description:
// Author:
// Modifications:

// webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
// standard libraries
#include <iostream>
#include <array>
#include <string>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>

constexpr double MAX_MOTOR_SPEED {10};              // Max speed of the robot motor in rad/s
constexpr int DISTANCE_SENSOR_NUMBER {4};           // number of distance sensors
constexpr double FAR_OBS_THRESHOLD {800.0};        // Threshold for detecting obstacle
constexpr double NEAR_OBS_THRESHOLD {500.0};        // Threshold for detecting obstacle
constexpr int COLOUR_THRESHOLD {3};
constexpr double SLOW_SPEED {0.8 * MAX_MOTOR_SPEED};
constexpr double FAST_SPEED {1.0 * MAX_MOTOR_SPEED};
constexpr double TURN_SPEED {0.1 * MAX_MOTOR_SPEED};
constexpr long double PI {3.141592653589793238462643383279502884L};

class TaskManager {
public:
    TaskManager(std::string name, std::string rName, int tStep, std::string mode = "start")
        : mName {name}, mRobotName {rName}, mMode {mode}, mCommand {""}, mTimeStep {tStep}, mSTModeIndex {0}, mRotations {0.0, 0.0},
        mSTModeWalls {'r', 'n', 'n', 'n','r', 'n', 'n', 'n', 'l', 'n','l', 'n', 'n'},
        mSTModeStages { {2250.0,2250.0}, {710, -710}, {3700.0, 3700.0}, {710, -710}, {3400.0, 3400.0}, {705, -705}, {9400, 9400}, {-570, 570}, {3200.0, 3200.0}, {-800, 800}, {3400.0, 3400.0}, {-250, 250}, {3200, 3200} }, 
        mCmdsPrinted {false}, mTaskComplete {false} {};
    void printCommands (const double simTime);  //List available commands for current mode
    std::string getMode() const;
    std::string getCommand (const webots::Keyboard *kb, const int input); //Get user command input
    std::vector<double> getSTModeSpeeds(bool wallLeft, bool wallAhead, bool wallRight); //Calculates the speed at which to run each motor in shortest-time mode
    void setCmdsPrinted(const bool state);
    void setTaskComplete(const bool state);
    bool isTaskComplete() const;
private:
    std::string mName;
    std::string mRobotName;
    std::string mMode; //Current operating mode
    std::string mCommand;
    int mTimeStep;
    int mSTModeIndex;
    std::vector<double> mRotations;
    std::vector<char> mSTModeWalls;
    std::vector<std::vector<double> > mSTModeStages;
    bool mCmdsPrinted; //Track whether command list for current mode has been printed
    bool mTaskComplete; //Track whether .csv has been processed for "read" mode
};

void TaskManager::printCommands(const double simTime) {
  if (!mCmdsPrinted) {
    if (mMode == "start") {
      std::cout << mName << ": Please select the command:\n";
      std::cout << mName << ": [1] run ROBOT_RED in manual-control mode and write data to ROBOT_RED.csv\n";
      std::cout << mName << ": [2] run ROBOT_BLUE in manual-control mode and write data to ROBOT_BLUE.csv\n";
      std::cout << mName << ": [3] run ROBOT_RED and ROBOT_BLUE in wall-following mode\n";
      std::cout << mName << ": [4] run ROBOT_RED or ROBOT_BLUE in shortest-time mode\n";
      std::cout << mName << ": [5] read data from ROBOT_RED.csv and process\n";
      std::cout << mName << ": [6] read data from ROBOT_BLUE.csv and process\n";
      mCmdsPrinted = true;
    } else if (mMode == "man") {
      std::cout << mName << ": Your input was " << (mRobotName == "ROBOT_RED" ? '1' : '2') << " - now run ROBOT_RED in manual-control mode and write data to ROBOT_RED.csv\n";
      std::cout << mRobotName << ": Starting writing data to ROBOT_RED.csv\n";
      std::cout << mRobotName << ": Please use the following commands to control the motion:\n";
      std::cout << mRobotName << ": " << (mRobotName == "ROBOT_RED" ? "[W]" : "[UP]") << " Move forward\n";
      std::cout << mRobotName << ": " << (mRobotName == "ROBOT_RED" ? "[S]" : "[DOWN]") << " Move backward\n";
      std::cout << mRobotName << ": " << (mRobotName == "ROBOT_RED" ? "[A]" : "[LEFT]") << " Turn left\n";
      std::cout << mRobotName << ": " << (mRobotName == "ROBOT_RED" ? "[D]" : "[RIGHT]") << " Turn right\n";
      std::cout << mRobotName << ": [SPACE] Stop\n";
      mCmdsPrinted = true;
    } else if (mMode == "follow") {
      if (mTaskComplete) {
        std::cout << mRobotName << ": Reaching target position in wall-following mode at " << simTime << " s\n";
      } else {
        std::cout << mName << ": Your input was 3 - now run ROBOT_RED and ROBOT_BLUE in wall-following mode\n";
        std::cout << mRobotName << ": Starting wall-following mode at " << simTime << " s\n";
      }
      mCmdsPrinted = true;
    } else if (mMode == "short") {
      if (mTaskComplete) {
        std::cout << mRobotName << ": Reaching target position in shortest-time mode at " << simTime << " s\n";
      } else {
        std::cout << mName << ": Your input was 4 - now run ROBOT_RED or ROBOT_BLUE in shortest-time mode\n";
        std::cout << mRobotName << ": Starting shortest-time mode at " << simTime << " s\n";
      }
      mCmdsPrinted = true;
    } else if (mMode == "read") {
      std::cout << mName << ": Your input was " << (mRobotName == "ROBOT_RED" ? '5' : '6') << " - now read data from " << mRobotName << ".csv and process\n";
      mCmdsPrinted = true;
    } else {
      std::cout << mName << ": Invalid mMode value (" << mMode << ") given to printCommands function";
      mCmdsPrinted = true;
    }
  }
}

std::string TaskManager::getMode() const {
  return mMode;
}

std::string TaskManager::getCommand(const webots::Keyboard *kb, const int input) {
  if (mMode == "start") {
    switch (input) {
      case '1':
        if (mRobotName == "ROBOT_RED") {
          mMode = "man";
          mCmdsPrinted = false;
        }
        break;
      case '2':
        if (mRobotName == "ROBOT_BLUE") {
          mMode = "man";
          mCmdsPrinted = false;
        }
        break;
      case '3':
        mMode = "follow";
        mCmdsPrinted = false;
        break;
      case '4':
        if (mRobotName == "ROBOT_BLUE") {
          mMode = "short";
          mCmdsPrinted = false;
        }
        break;
      case '5':
        if (mRobotName == "ROBOT_RED") {
          mMode = "read";
          mCmdsPrinted = false;
        }
        break;
      case '6':
        if (mRobotName == "ROBOT_BLUE") {
          mMode = "read";
          mCmdsPrinted = false;
        }
        break;
    }
  } else if (mMode == "man") {
    if (mRobotName == "ROBOT_RED") {
      switch (input) {
        case 'W':
          mCommand = "forward";
          break;
        case 'S':
          mCommand = "backward";
          break;
        case 'A':
          mCommand = "left";
          break;
        case 'D':
          mCommand = "right";
          break;
        case ' ':
          mCommand = "pause";
          break;
      }
    } else {
      switch (input) {
        case (kb->UP):
          mCommand = "forward";
          break;
        case (kb->DOWN):
          mCommand = "backward";
          break;
        case (kb->LEFT):
          mCommand = "left";
          break;
        case (kb->RIGHT):
          mCommand = "right";
          break;
        case ' ':
          mCommand = "pause";
          break;
      }
    }
  }
  return mCommand;
}

std::vector<double> TaskManager::getSTModeSpeeds(bool wallLeft, bool wallAhead, bool wallRight) {
  std::vector<double> speeds {0.0, 0.0};
  double lRotationTarget = mSTModeStages[mSTModeIndex][0];
  double rRotationTarget = mSTModeStages[mSTModeIndex][1];
  bool leftCorrect = (wallAhead || (wallRight && !wallLeft)) && mSTModeWalls[mSTModeIndex] == 'l'; //Determine whether robot has turned too far to the left
  bool rightCorrect = (wallAhead || (wallLeft && !wallRight)) && mSTModeWalls[mSTModeIndex] == 'r'; //Determine whether robot has turned too far to the right

  if (mRotations[0] == lRotationTarget && mRotations[1] == rRotationTarget) { //If end of stage has been reached, reset rotation counters and increment index
    std::fill(mRotations.begin(), mRotations.end(), 0);
    if (mSTModeIndex < (int)mSTModeStages.size()-1) {
      mSTModeIndex++;
    } else {
      mTaskComplete = true;
      mCmdsPrinted = false;
    }
    return speeds;
  }

  double lGapSpeed = (double)abs((lRotationTarget - mRotations[0]) * 2 * PI / mTimeStep); //Calculate speed required to bridge the gap between left motor's completed and target rotations
  double rGapSpeed = (double)abs((rRotationTarget - mRotations[1]) * 2 * PI / mTimeStep); //Calculate speed required to bridge the gap between right motor's completed and target rotations

  speeds[0] = ((lRotationTarget < 0 || rightCorrect) ? -1 : 1) * std::min(rightCorrect ? SLOW_SPEED : FAST_SPEED, lGapSpeed); //Set left motor speed according to difference between completed rotations and target rotations
  speeds[1] = ((rRotationTarget < 0 || leftCorrect) ? -1 : 1) * std::min(leftCorrect ? SLOW_SPEED : FAST_SPEED, rGapSpeed); //Set right motor speed according to difference between completed rotations and target rotations

  if (!leftCorrect && !rightCorrect) {
    mRotations[0] += mTimeStep * speeds[0] / (2 * PI);
    mRotations[1] += mTimeStep * speeds[1] / (2 * PI);
  }

  return speeds;
}

void TaskManager::setCmdsPrinted(const bool state) {
  mCmdsPrinted = state;
}

void TaskManager::setTaskComplete(const bool state) {
  mTaskComplete = state;
}

bool TaskManager::isTaskComplete() const {
  return mTaskComplete;
}

class speedController {
public:
  speedController()
    : mMode{"start"};
  void setMode() const;
  std::vector<int> getSpeeds();
  
private:
  std::string mMode;
};

// a class to write/read data to/from csv file
class CsvProcessor {
public:
    CsvProcessor(std::string robotName, std::string fileName, char delim = ',')           // constructor
        : mRobotName {robotName}, mFileName {fileName}, mDelim {delim} {};                // default delim = ','
    void writeLineToCsv (const std::vector<double> &dataLine) const;                      // write line to csv
    std::vector<std::vector<double> > readDataFromCsv() const;                            // read data (multiple lines) from csv
    std::vector<std::vector<double> > rProcessData(std::vector<std::vector<double> > dataMatrix) const;  // process a 2D vector<double> of data
    void bProcessData(std::vector<std::vector<double> > &dataMatrix) const;
private:
    std::string mRobotName;
    std::string mFileName;                                                        // csv file name
    char mDelim;                                                                  // delimiter
};

// write line to csv file
void CsvProcessor::writeLineToCsv (const std::vector<double> &dataLine) const {
    std::ofstream fout {mFileName, std::ios::out|std::ios::app};  // open file in append mode
    if (fout.is_open()) {  // check if open successfully
        // write elements (except the last) of dataLine to file, delimited by ','
        for (auto iter = dataLine.begin(); iter != dataLine.end()-1; ++iter) { 
            fout << std::fixed << std::setprecision(3) << *iter << mDelim;      
        }
        // write the last element of dataLine to file, followed by '\n' and flush
        fout << std::fixed << std::setprecision(3) << dataLine.back() << std::endl;
    } else {
        // throw an error message
        throw std::runtime_error (mRobotName + ": Writing new line to " + mFileName + " failed!");
    }
}

// read data (multiple lines) from csv file
// return a 2D vector of double
std::vector<std::vector<double> > CsvProcessor::readDataFromCsv() const {
    std::vector<std::vector<double> > dataMatrix; //2D vector<double> to store lines of data read from the .csv file

    std::ifstream fIn(mFileName, std::ios::in);
    if (fIn) { //Check whether file is open
        std::string dataLine;

        while (std::getline(fIn, dataLine)) { //While the .csv file has more lines to be read
            std::stringstream sStream{ dataLine };

            std::vector<double> data;

            while (sStream.good()) { //While the line has more data elements to be read
                std::string dataString;
                std::getline(sStream, dataString, mDelim);
                data.push_back(std::stod(dataString)); //Push data element, converted from string to double, to the vector<double> data
            }
            dataMatrix.push_back(data); //Push complete vector<double> data to back of 2D vector<double> dataMatrix
        }
        fIn.close();
        std::cout << mRobotName << ": Reading data from " << mFileName << " succeeded!\n"; //.csv file has been fully read, notify user
    } else { //If file has failed to open, throw an error
        throw std::runtime_error (mRobotName + ": Reading data from " + mFileName + " failed!\n");
    }
    return dataMatrix;
  }

std::vector<std::vector<double> > CsvProcessor::rProcessData(std::vector<std::vector<double> > dataMatrix) const {
  std::vector<double> maxVals;
  std::vector<double> minVals;
  std::vector<double> valSums;
  auto firstRow {*dataMatrix.begin()};
  for (auto col = firstRow.begin(); col != firstRow.end(); col++) { //Initial values for all data vectors should be first row of data
    maxVals.push_back(*col);
    minVals.push_back(*col);
    valSums.push_back(*col);
  }

  for (auto row = std::next(dataMatrix.begin()); row != dataMatrix.end(); row++) { //Iterate through rows and columns of data matrix, starting from the second
      for (auto col = row->begin(); col != row->end(); col++) {
        int index = col - row->begin();
        if (*col > maxVals[index]) { //Accumulate the maximum value in each column
          maxVals[index] = *col;
        }
        if (*col < minVals[index]) { //Accumulate the minimum value in each column
          minVals[index] = *col;
        }
        valSums[index] += *col; //Accumulate the sum of all values in each column
      }
  }
  
  std::vector<double> avgVals;
  int nRows {(int)dataMatrix.size()};
  for (auto iter = valSums.begin(); iter != valSums.end(); iter++) { //Calculate average of each colum from sum of each column and number of rows
    double avgVal = *iter / nRows;
    avgVals.push_back(avgVal);
  }
  
  std::vector<std::vector<double> > processedData;
  processedData.push_back(maxVals);
  processedData.push_back(minVals);
  processedData.push_back(avgVals);
  return processedData;
}

void CsvProcessor::bProcessData(std::vector<std::vector<double> > &dataMatrix) const {
  std::sort(dataMatrix.begin(), dataMatrix.end(), [](const std::vector<double>& lhs, const std::vector<double>& rhs) {
                  return lhs.back() < rhs.back();
              });
  std::string fName = mRobotName + "_sorted.csv";
  std::ofstream fout {fName, std::ios::out|std::ios::app};  // open file in append mode
  if (fout.is_open()) {  // check if open successfully
    for (auto row = dataMatrix.begin(); row != dataMatrix.end(); row++) { //Write each row of sorted values to the output file
        for (auto col = row->begin(); col != row->end(); col++) {
          fout << std::fixed << std::setprecision(3) << *col << mDelim;
        }      
        fout << std::fixed << std::setprecision(3) << std::endl;
    }
    std::cout << mRobotName << ": Writing data to " << fName << " succeeded!\n";
  } else {
      // throw an error message
      throw std::runtime_error (mRobotName + ": Writing data to " + fName + " failed!\n");
  }
}

int main(int argc, char **argv) {
  // create the Robot instance.
  webots::Robot robot; 

  // get the time step of the current world.
  int timeStep = (int)robot.getBasicTimeStep();
  
  std::array<webots::DistanceSensor*, DISTANCE_SENSOR_NUMBER> ds;
  std::array<std::string, DISTANCE_SENSOR_NUMBER> dsNames {
      "ds0", "ds1", "ds2", "ds3"
  };
  for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i) {
      ds[i] = robot.getDistanceSensor(dsNames[i]);
      ds[i]->enable(timeStep);
  }
    
  // get the motor devices and initialise
  webots::Motor *leftMotor = robot.getMotor("left wheel motor");
  webots::Motor *rightMotor = robot.getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // get the name of the robot
  std::string robotName = robot.getName();
  
  // get the camera device and enable
  webots::Camera *camera = robot.getCamera("camera");
  camera->enable(timeStep);
  int width = camera->getWidth();
  int height = camera->getHeight();
  
  // get keyboard and enable
  webots::Keyboard *keyboard = robot.getKeyboard();
  keyboard->enable(timeStep);
  int key {-1}; // initialise key with -1 (no key input)

  TaskManager tMan {"TASK_MANAGER", robotName, timeStep};

  double leftSpeed  = 0.0;
  double rightSpeed = 0.0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot.step(timeStep) != -1) {
    std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;
    for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++) {
      dsValues[i] = ds[i]->getValue();
    }

    bool greenFound = false;
    // get image from camera and detect blob
    const unsigned char *image = camera->getImage();
    if (image) { // image may be NULL if Robot.synchronization is FALSE
        int red {0};
        int green {0};
        int blue {0};
        
        // Here we analyse the image from the camera. The goal is to detect a
        // blob (a spot of color) of a defined color in the middle of our
        // screen.
        // In order to achieve that we simply parse the image pixels of the
        // center of the image, and sum the color components individually
        for (int i = width / 3; i < 2 * width / 3; ++i) {
            for (int j = height / 2; j < 3 * height / 4; ++j) {
                red += camera->imageGetRed(image, width, i, j);
                blue += camera->imageGetBlue(image, width, i, j);
                green += camera->imageGetGreen(image, width, i, j);
            }
        }
        
        // If a component is much more represented than the other ones,
        // a blob is detected
        if ((green > COLOUR_THRESHOLD * red) && (green > COLOUR_THRESHOLD * blue)) {
            greenFound = true;
        }
    }

    key = keyboard->getKey();
    double time = robot.getTime();

    tMan.printCommands(time);
    std::string mode = tMan.getMode();
    std::string command = tMan.getCommand(keyboard, key);
    if (mode == "man") { //Manual control mode
      std::string fileName = robotName + ".csv";
      std::vector<double> dataLine; // use a vector of double to store the data 
      dataLine.push_back(time);
      dataLine.insert(dataLine.end(), dsValues.begin(), dsValues.end());
      try {
          CsvProcessor csvProcessor{robotName, fileName}; // create a CsvProcessor
          csvProcessor.writeLineToCsv(dataLine); // write new line to csv file 
          // print the new line written to csv file
          for (std::size_t i = 0; i < dataLine.size(); ++i) {
              std::cout << std::fixed << std::setprecision(3) << dataLine[i] << " ";
          }
          std::cout << std::endl;
      } catch(const std::runtime_error &e) {
          std::cerr << e.what() << std::endl;
      }
      if (command == "forward") {
        leftSpeed  = SLOW_SPEED;
        rightSpeed = SLOW_SPEED;
      } else if (command == "backward") {
        leftSpeed  = -SLOW_SPEED;
        rightSpeed = -SLOW_SPEED;
      } else if (command == "left") {
        leftSpeed  = -TURN_SPEED;
        rightSpeed = SLOW_SPEED;
      } else if (command == "right") {
        leftSpeed  = SLOW_SPEED;
        rightSpeed = -TURN_SPEED;
      } else if (command == "pause") {
        leftSpeed  = 0.0;
        rightSpeed = 0.0;
      }
    } else if (mode == "read" && !tMan.isTaskComplete()) { //Read and process .csv file mode
      std::string fileName = robotName + ".csv";
      CsvProcessor csvProcessor{robotName, fileName}; // create a CsvProcessor
      if (robotName == "ROBOT_RED") {
        std::vector<std::vector<double> > pData = csvProcessor.rProcessData(csvProcessor.readDataFromCsv());
        const std::vector<std::string> vLabels {"maximum", "minimum", "average"}; //Strings describing the type of processed data printed to cout
        for (auto row = pData.begin(); row != pData.end(); row++) { //Iterate through rows and columns of data matrix
            int index = row - pData.begin();
            std::cout << robotName << ": The " << vLabels[index] << " values are - "; //Print descriptive string at start of data line
            for (auto col = row->begin(); col != row->end()-1; col++) {
                std::cout << std::fixed << std::setprecision(3) << *col << ", "; //Print processed data row by row, separated by spaces
            }
            std::cout << std::fixed << std::setprecision(3) << *(row->end()-1) << std::endl;
        }
      } else {
        std::vector<std::vector<double> > dataMatrix = csvProcessor.readDataFromCsv();
        csvProcessor.bProcessData(dataMatrix);
      }
      tMan.setTaskComplete(true);
    } else if (mode == "follow" && !tMan.isTaskComplete()) { //Wall-follow mode
      bool wallLeft = dsValues[3] > FAR_OBS_THRESHOLD;
      bool wallRight = dsValues[0] > FAR_OBS_THRESHOLD;
      bool wallAhead = dsValues[1] > FAR_OBS_THRESHOLD || dsValues[2] > FAR_OBS_THRESHOLD;
      if (robotName == "ROBOT_RED") {
        if (greenFound) {
          leftSpeed  = 0.0;
          rightSpeed = 0.0;
          tMan.setTaskComplete(true);
          tMan.setCmdsPrinted(false);
        } else if (wallAhead) {
          leftSpeed  = SLOW_SPEED;
          rightSpeed = -TURN_SPEED;
        } else if (wallLeft) {
          leftSpeed  = SLOW_SPEED;
          rightSpeed = SLOW_SPEED;
        } else {
          leftSpeed  = -TURN_SPEED;
          rightSpeed = SLOW_SPEED;
        }
      } else {
        if (greenFound) {
          leftSpeed  = 0.0;
          rightSpeed = 0.0;
          tMan.setTaskComplete(true);
          tMan.setCmdsPrinted(false);
        } else if (wallAhead) {
          leftSpeed  = -TURN_SPEED;
          rightSpeed = SLOW_SPEED;
        } else if (wallRight) {
          leftSpeed  = SLOW_SPEED;
          rightSpeed = SLOW_SPEED;
        } else {
          leftSpeed  = SLOW_SPEED;
          rightSpeed = -TURN_SPEED;
        }
      }
    } else if (mode == "short" && !tMan.isTaskComplete()) {
      bool wallLeft = dsValues[3] > NEAR_OBS_THRESHOLD;
      bool wallRight = dsValues[0] > NEAR_OBS_THRESHOLD;
      bool wallAhead = dsValues[1] > FAR_OBS_THRESHOLD || dsValues[2] > FAR_OBS_THRESHOLD;
      std::vector<double> motorSpeeds = tMan.getSTModeSpeeds(wallLeft, wallAhead, wallRight);
      leftSpeed = motorSpeeds[0];
      rightSpeed = motorSpeeds[1];
    }
    //write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  };

  return 0;
}
