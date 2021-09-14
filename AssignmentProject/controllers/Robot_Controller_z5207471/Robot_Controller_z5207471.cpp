// File:          Robot_Controller_z5207471.cpp
// Date:          5/10/20
// Description:   Robot controller program written for MTRN2500 T3 2020 Assignment 1
// Author:        Lachlan Scott (z5207471)
// Modifications: None, First version

//Webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
//Standard libraries
#include <iostream>
#include <array>
#include <string>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
//Constant values
constexpr int MOTOR_NUMBER {2};                                     // Number of motors
constexpr int DISTANCE_SENSOR_NUMBER {4};                           // Number of distance sensors
constexpr int COLOUR_THRESHOLD {5};                                 // Threshold for determining a colour majority in a pixel array
constexpr double MAX_MOTOR_SPEED {10.0};                            // Max speed of the robot motor in rad/s
constexpr double NEAR_OBS_THRESHOLD {700.0};                        // Sensor reading threshold for detecting close obstacles
constexpr double FAR_OBS_THRESHOLD {600.0};                         // Sensor reading threshold for detecting distant obstacles
constexpr double MIN_OBS_THRESHOLD {400.0};                         // Minimum sensor reading threshold for detecting obstacles 
constexpr double MAX_OBS_THRESHOLD {800.0};                         // Maximum sensor reading threshold for detecting obstacles 
constexpr double SLOW_SPEED {0.5 * MAX_MOTOR_SPEED};                // Slow speed setting for a motor
constexpr double FAST_SPEED {1.0 * MAX_MOTOR_SPEED};                // Fast speed setting for a motor
constexpr double CORRECTION_SPEED {0.95 * MAX_MOTOR_SPEED};         // Speed setting for slight adjustment to robot path
constexpr double TURN_SPEED {0.1 * MAX_MOTOR_SPEED};                // Turning speed setting for a motor

enum class opMode { // Define easily legible symbols for each controller operation mode
  Start, Manual, Follow, Short, Read
};

enum class manCommand { // Define easily legible symbols for each manual-control mode command
  Forward, Backward, Left, Right, Pause
};

enum class stMove { // Define easily legible symbols for each type of movement operation in the shortest-time mode
  lWall, aWall, rWall, lTurn, rTurn
};

// A class used to handle printing output to user, reading of command inputs and tracking task completion
class TaskManager {
public:
    TaskManager()
        : mCmdsPrinted {false}, mTaskComplete {false} {};
    void printOutput(const std::string robotName, const opMode mode, const double simTime);  // List available commands for current mode
    opMode readCommand (const opMode currentMode, std::string rName, const int input);       // Read user command input to select operating mode
    manCommand readManCmdRed(const manCommand currentCmd, const int input);                  // Read user manual-mode command input for red robot
    manCommand readManCmdBlue(const manCommand currentCmd, const int input);                 // Read user manual-mode command input for blue robot
    void setCmdsPrinted(const bool state);                                                   // Set the mCmdsPrinted flag, to determine whether command options have been printed yet
    void setTaskComplete(const bool state);                                                  // Set the mTaskComplete flag, to determine whether the current task is complete yet
    bool isTaskComplete() const;                                                             // Return the state of the mTaskComplete flag
private:
    bool mCmdsPrinted;                                                                       // Track whether command list for current mode has been printed
    bool mTaskComplete;                                                                      // Track whether .csv has been processed for "read" mode
};

// Print appropriate user prompts or useful information based upon robot name, operating mode and simulation time
void TaskManager::printOutput(const std::string robotName, const opMode mode, const double simTime) {
  if (!mCmdsPrinted) { // Only print a set of commands/information once
    switch (mode) { // Print correct information for current mode
      case opMode::Start:
        std::cout << "TASK_MANAGER: Please select the command:\n";
        std::cout << "TASK_MANAGER: [1] run ROBOT_RED in manual-control mode and write data to ROBOT_RED.csv\n";
        std::cout << "TASK_MANAGER: [2] run ROBOT_BLUE in manual-control mode and write data to ROBOT_BLUE.csv\n";
        std::cout << "TASK_MANAGER: [3] run ROBOT_RED and ROBOT_BLUE in wall-following mode\n";
        std::cout << "TASK_MANAGER: [4] run ROBOT_RED or ROBOT_BLUE in shortest-time mode\n";
        std::cout << "TASK_MANAGER: [5] read data from ROBOT_RED.csv and process\n";
        std::cout << "TASK_MANAGER: [6] read data from ROBOT_BLUE.csv and process\n";
        mCmdsPrinted = true;
      break;
      case opMode::Manual:
        std::cout << "TASK_MANAGER: Your input was " << (robotName == "ROBOT_RED" ? '1' : '2') << " - now run " + robotName + " in manual-control mode and write data to " + robotName + ".csv\n";
        std::cout << robotName << ": Starting writing data to " + robotName + ".csv\n";
        std::cout << robotName << ": Please use the following commands to control the motion:\n";
        std::cout << robotName << ": " << (robotName == "ROBOT_RED" ? "[W]" : "[UP]") << " Move forward\n";
        std::cout << robotName << ": " << (robotName == "ROBOT_RED" ? "[S]" : "[DOWN]") << " Move backward\n";
        std::cout << robotName << ": " << (robotName == "ROBOT_RED" ? "[A]" : "[LEFT]") << " Turn left\n";
        std::cout << robotName << ": " << (robotName == "ROBOT_RED" ? "[D]" : "[RIGHT]") << " Turn right\n";
        std::cout << robotName << ": [SPACE] Stop\n";
        mCmdsPrinted = true;
      break;
      case opMode::Follow:
        if (mTaskComplete) {
          std::cout << robotName << ": Reaching target position in wall-following mode at " << simTime << " s\n";
        } else {
          std::cout << "TASK_MANAGER: Your input was 3 - now run ROBOT_RED and ROBOT_BLUE in wall-following mode\n";
          std::cout << robotName << ": Starting wall-following mode at " << simTime << " s\n";
        }
        mCmdsPrinted = true;
      break;
      case opMode::Short:
        if (mTaskComplete) {
          std::cout << robotName << ": Reaching target position in shortest-time mode at " << simTime << " s\n";
        } else {
          std::cout << "TASK_MANAGER: Your input was 4 - now run ROBOT_RED or ROBOT_BLUE in shortest-time mode\n";
          std::cout << robotName << ": Starting shortest-time mode at " << simTime << " s\n";
        }
        mCmdsPrinted = true;
      break;
      case opMode::Read:
        std::cout << "TASK_MANAGER: Your input was " << (robotName == "ROBOT_RED" ? '5' : '6') << " - now read data from " << robotName << ".csv and process\n";
        mCmdsPrinted = true;
      break;
      default: // If an invalid opMode value is somehow encountered, throw a descriptive error message
        throw std::invalid_argument("TASK_MANAGER: Invalid opMode value given to printCommands function!");
        mCmdsPrinted = true;
    }
  }
}

// Read user input to allow user to select an operating mode
opMode TaskManager::readCommand(const opMode currentMode, std::string rName, const int input) {
  opMode newMode {currentMode};
  switch (input) {
    case '1': // If the user pressed 1, set ROBOT_RED's operating mode to manual control
      if (rName == "ROBOT_RED") {
        newMode = opMode::Manual;
        mCmdsPrinted = false;
      }
      break;
    case '2': // If the user pressed 2, set ROBOT_BLUE's operating mode to manual control
      if (rName == "ROBOT_BLUE") {
        newMode = opMode::Manual;
        mCmdsPrinted = false;
      }
      break;
    case '3': // If the user pressed 3, set the robot's operating mode to wall-following
      newMode = opMode::Follow;
      mCmdsPrinted = false;
      break;
    case '4': // If the user pressed 4, set ROBOT_BLUE's operating mode to shortest-time
      if (rName == "ROBOT_BLUE") {
        newMode = opMode::Short;
        mCmdsPrinted = false;
      }
      break;
    case '5': // If the user pressed 5, set ROBOT_RED's operating mode to read and process data
      if (rName == "ROBOT_RED") {
        newMode = opMode::Read;
        mCmdsPrinted = false;
      }
      break;
    case '6': // If the user pressed 5, set ROBOT_BLUE's operating mode to read and process data
      if (rName == "ROBOT_BLUE") {
        newMode = opMode::Read;
        mCmdsPrinted = false;
      }
      break;
  }
  return newMode;
}

// For ROBOT_RED, read user input to determine current manual-mode command
manCommand TaskManager::readManCmdRed(const manCommand currentCmd, const int input) {
  manCommand newCmd = currentCmd;
  switch (input) {
    case 'W': // If user pressed the W key, set command to Forward
      newCmd = manCommand::Forward;
      break;
    case 'S': // If user pressed the S key, set command to Backward
      newCmd = manCommand::Backward;
      break;
    case 'A': // If user pressed the A key, set command to Left
      newCmd = manCommand::Left;
      break;
    case 'D': // If user pressed the D key, set command to Right
      newCmd = manCommand::Right;
      break;
    case ' ': // If user pressed the space key, set command to Pause
      newCmd = manCommand::Pause;
      break;
  }
  return newCmd;
}

// For ROBOT_BLUE, read user input to determine current manual-mode command
manCommand TaskManager::readManCmdBlue(const manCommand currentCmd, const int input) {
  manCommand newCmd = currentCmd;
  switch (input) {
    case (webots::Keyboard::UP): // If user pressed the up arrow key, set command to Forward
      newCmd = manCommand::Forward;
      break;
    case (webots::Keyboard::DOWN): // If user pressed the down arrow key, set command to Backward
      newCmd = manCommand::Backward;
      break;
    case (webots::Keyboard::LEFT): // If user pressed the left arrow key, set command to Left
      newCmd = manCommand::Left;
      break;
    case (webots::Keyboard::RIGHT): // If user pressed the right arrow key, set command to Right
      newCmd = manCommand::Right;
      break;
    case ' ':
      newCmd = manCommand::Pause; // If user pressed the space key, set command to Pause
      break;
  }
  return newCmd;
}

// Set current state of mCmdsPrinted boolean flag
void TaskManager::setCmdsPrinted(const bool state) {
  mCmdsPrinted = state;
}

// Set current state of mTaskComplete boolean flag
void TaskManager::setTaskComplete(const bool state) {
  mTaskComplete = state;
}

// Get current state of mTaskComplete boolean flag
bool TaskManager::isTaskComplete() const {
  return mTaskComplete;
}

// A class used to determine the appropriate speed setting for each motor based upon current operating mode, user input and sensor readings
class SpeedController {
public:
  SpeedController() // Constructor
    : mSTModeStages{ stMove::rWall, stMove::rTurn, stMove::rWall, stMove::rTurn, stMove::rWall, stMove::aWall, stMove::lWall, stMove::lTurn, stMove::lWall, stMove::lTurn, stMove::lWall, stMove::aWall, stMove::rWall },
    mSTModeIndex{0} {};
  std::array<double, MOTOR_NUMBER> getMCModeSpds(manCommand cmd) const;                                                     // Get motor speeds for manual-control mode
  std::array<double, MOTOR_NUMBER> getWFModeSpdsRed(const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const;      // Get motor speeds for ROBOT_RED in wall-following mode
  std::array<double, MOTOR_NUMBER> getWFModeSpdsBlue(const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const;     // Get motor speeds for ROBOT_BLUE in wall-following mode
  std::array<double, MOTOR_NUMBER> getSTModeSpds(int timeStep, const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues); // Get motor speeds for shortest-time mode
private:
  const std::vector<stMove> mSTModeStages; // Stores stages in the shortest-time mode movement sequence
  int mSTModeIndex;                        // Tracks current stage in the shortest-time mode movement sequence
};

// Determine appropriate motor speeds based upon current manual-mode command
std::array<double, MOTOR_NUMBER> SpeedController::getMCModeSpds(manCommand cmd) const {
  std::array<double, MOTOR_NUMBER> speeds;
  switch (cmd) { // Check current manual command
    case manCommand::Forward: // If Forward command is given, move forward
      speeds[0] = SLOW_SPEED;
      speeds[1] = SLOW_SPEED;
    break;
    case manCommand::Backward: // If Backward command is given, move backward
      speeds[0] = -SLOW_SPEED;
      speeds[1] = -SLOW_SPEED;
    break;
    case manCommand::Left: // If Left command is given, move left
      speeds[0] = -SLOW_SPEED;
      speeds[1] = SLOW_SPEED;
    break;
    case manCommand::Right: // If Right command is given, move right
      speeds[0] = SLOW_SPEED;
      speeds[1] = -SLOW_SPEED;
    break;
    case manCommand::Pause: // If pause command is given, set motor speeds to 0.0
      return speeds;
    break;
    default: // If an invalid manCommand value is somehow encountered, throw a descriptive error message
      throw std::invalid_argument("Invalid manual command given to speedController::getMCModeSpds!");
  }
  return speeds;
}

// For ROBOT_RED, determine motor speeds for current stage in wall-following mode operation
std::array<double, MOTOR_NUMBER> SpeedController::getWFModeSpdsRed(const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const {
  std::array<double, MOTOR_NUMBER> speeds;
  bool wallLeft = dsValues[3] > NEAR_OBS_THRESHOLD;
  bool wallAhead = dsValues[1] > FAR_OBS_THRESHOLD || dsValues[2] > FAR_OBS_THRESHOLD;
  if (wallAhead) { // If a wall is ahead, turn right (This takes priority over following a left wall)
    speeds[0] = SLOW_SPEED;
    speeds[1] = -TURN_SPEED;
  } else if (wallLeft) { // If there's a wall to the left, continue forward
    speeds[0] = SLOW_SPEED;
    speeds[1] = SLOW_SPEED;
  } else { // If there's no wall ahead or to the left, turn left
    speeds[0] = -TURN_SPEED;
    speeds[1] = SLOW_SPEED;
  }
  return speeds;
}

// For ROBOT_BLUE, determine motor speeds for current stage in wall-following mode operation
std::array<double, MOTOR_NUMBER> SpeedController::getWFModeSpdsBlue(const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const {
  std::array<double, MOTOR_NUMBER> speeds;
  bool wallAhead = dsValues[1] > FAR_OBS_THRESHOLD || dsValues[2] > FAR_OBS_THRESHOLD;
  bool wallRight = dsValues[0] > NEAR_OBS_THRESHOLD;
  if (wallAhead) { // If a wall is ahead, turn left (This takes priority over following a left wall)
    speeds[0] = -TURN_SPEED;
    speeds[1] = SLOW_SPEED;
  } else if (wallRight) { // If there's a wall to the right, continue forward
    speeds[0] = SLOW_SPEED;
    speeds[1] = SLOW_SPEED;
  } else { // If there's no wall ahead or to the right, turn right
    speeds[0] = SLOW_SPEED;
    speeds[1] = -TURN_SPEED;
  }
  return speeds;
}

// Determine motor speeds for current stage in shortest-time mode operation
// Uses a hybrid between hard-coded and dynamic techniques to balance optimisation with reliablity
std::array<double, MOTOR_NUMBER> SpeedController::getSTModeSpds(int timeStep, const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) {
  std::array<double, MOTOR_NUMBER> speeds {0.0, 0.0};

  bool wallLeft = dsValues[3] > FAR_OBS_THRESHOLD;                                              // Determine whether there is a wall within the desired distance from the robot's left side
  bool wallLeftDistant = dsValues[3] <= FAR_OBS_THRESHOLD && dsValues[3] > MIN_OBS_THRESHOLD;   // Determine whether there is a wall that is currently too far from the robot's left side
  bool wallLeftClose = dsValues[3] >= MAX_OBS_THRESHOLD;                                        // Determine whether there is a wall that is currently too close to the robot's left side

  bool wallAheadLeft = dsValues[2] > NEAR_OBS_THRESHOLD;                                        // Determine whether there is a wall ahead and to the left of the robot
  bool wallAhead = dsValues[2] > FAR_OBS_THRESHOLD || dsValues[1] > FAR_OBS_THRESHOLD;          // Determine whether there is a wall ahead of the robot
  bool wallAheadRight = dsValues[1] > NEAR_OBS_THRESHOLD;                                       // Determine whether there is a wall ahead and to the right of the robot

  bool wallRight = dsValues[0] > FAR_OBS_THRESHOLD;                                             // Determine whether there is a wall within the desired distance from the robot's right side
  bool wallRightDistant = dsValues[0] <= FAR_OBS_THRESHOLD && dsValues[0] > MIN_OBS_THRESHOLD;  // Determine whether there is a wall that is currently too far from the robot's right side
  bool wallRightClose = dsValues[0] >= MAX_OBS_THRESHOLD;                                       // Determine whether there is a wall that is currently too close to the robot's right side

  bool stageComplete = true;

  while (stageComplete) { // If a stage in the process has been completed, get instructions for the next stage in the same time step, to improve speed
    switch(mSTModeStages[mSTModeIndex]) { // Check which movement should be made for the current stage
      case stMove::lWall:  // Move forward, until there is no longer a wall to the left
        if (wallAheadLeft) { // If there is a wall ahead and to the left, turn right to correct path
          speeds[0] = FAST_SPEED;
          speeds[1] = -FAST_SPEED;
          stageComplete = false;
        } else if (wallLeftClose) { // If there is a wall too close on the left, turn right very slightly to correct path
          speeds[0] = FAST_SPEED;
          speeds[1] = CORRECTION_SPEED;
          stageComplete = false;
        } else if (wallLeft) { // If there is a wall within the desired distance on the left, continue forward at maximum speed
          speeds[0] = FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        } else if (wallLeftDistant) { // If there is a wall too far away on the left, turn left very slightly to correct path
          speeds[0] = CORRECTION_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        }
      break;
      case stMove::aWall: // Move forward until a wall is encountered ahead
        if (!wallAhead) { //If a wall is not detected ahead, continue forward at maximum speed
          speeds[0] = FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        }
      break;
      case stMove::rWall: // Move forward, until there is no longer a wall to the right
        if (wallAheadRight) { // If there is a wall ahead and to the right, turn left to correct path
          speeds[0] = -FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        } else if (wallRightClose) { // If there is a wall too close on the right, turn left very slightly to correct path
          speeds[0] = CORRECTION_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        } else if (wallRight) { // If there is a wall within the desired distance on the right, continue forward at maximum speed
          speeds[0] = FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        } else if (wallRightDistant) { // If there is a wall too far away on the right, turn right very slightly to correct path
          speeds[0] = FAST_SPEED;
          speeds[1] = CORRECTION_SPEED;
          stageComplete = false;
        }
      break;
      case stMove::lTurn: // Perform a left turn until the adjacent left wall is encountered to ensure that a full 90-degree turn is reliably executed
        if (!wallAheadLeft) { // If the adjacent left wall has not been encountered, turn left
          speeds[0] = SLOW_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        } else if (!wallLeft) { // If a wall to the left has not been detected, continue forward until it has
          speeds[0] = FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        }
      break;
      case stMove::rTurn: // Perform a right turn until the adjacent right wall is encountered to ensure that a full 90-degree turn is reliably executed
        if (!wallAheadRight) { // If the adjacent left wall has not been encountered, turn left
          speeds[0] = FAST_SPEED;
          speeds[1] = SLOW_SPEED;
          stageComplete = false;
        } else if (!wallRight) { // If a wall to the left has not been detected, continue forward until it has
          speeds[0] = FAST_SPEED;
          speeds[1] = FAST_SPEED;
          stageComplete = false;
        }
      break;
      default: // If an invalid stMove value has somehow been encountered, throw a descriptive error
        throw std::invalid_argument("Invalid shortest-time mode movement operation given to speedController::getSTModeSpds!");
    }
    if (stageComplete) { // If no path adjustments have been made, the current stage is complete. 
      if (mSTModeIndex == (int)mSTModeStages.size() - 1) { // If the current stage is the final one, do not increment mSTModeIndex, but exit the while loop
        stageComplete = false;
      } else { // If the current stage is not the final one, increment mSTModeIndex and begin the next stage
        mSTModeIndex++;
      }
    }
  }
  return speeds;
}

// A class to read/write data to/from a .csv file and process this data as required
// Code snippets taken and modified from Exercise 3
class CsvProcessor {
public:
  CsvProcessor() {};
  CsvProcessor(std::string name, std::string fileName, char delim = ',')                                          // Constructor
      : mRobotName{name}, mFileName{fileName}, mDelim{delim} {};                                                  // Default delim = ','
  void writeLineToCsv (const std::vector<double> &dataLine) const;                                                // Write a single line of data to a .csv file
  void writeData(double simTime, const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const;               // Write a vector of data to a .csv file
  std::vector<std::vector<double> > readDataFromCsv() const;                                                      // Read a single line of data from a .csv file
  std::vector<std::vector<double> > rProcessData(const std::vector<std::vector<double> > &dataMatrix) const;      // Process data for ROBOT_RED
  static bool compCol(const std::vector<double> &v1, const std::vector<double> &v2);                              // Compare the final values of two double vectors and return whether the first is smaller
  void bProcessData(std::vector<std::vector<double> > &dataMatrix) const;                                         // Process data for ROBOT_BLUE
  void rPrintData(const std::vector<std::vector<double> > &dataMatrix) const;                                     // Print processed data for ROBOT_RED
private:
  std::string mRobotName;                                                                                         // Robot name
  std::string mFileName;                                                                                          // .csv file name
  char mDelim;                                                                                                    // Delimiter character
};

// Write line to csv file
// Code snippet taken from Exercise 3
void CsvProcessor::writeLineToCsv (const std::vector<double> &dataLine) const {
  std::ofstream fout {mFileName, std::ios::out|std::ios::app};  // Open file in append mode
  if (fout.is_open()) {  // Check whether the file was opened successfully
    // Write elements (except the last) of dataLine to file, delimited by ','
    for (auto iter = dataLine.begin(); iter != dataLine.end()-1; ++iter) { 
        fout << std::fixed << std::setprecision(3) << *iter << mDelim;      
    }
    // Write the last element of dataLine to file, followed by '\n' and flush
    fout << std::fixed << std::setprecision(3) << dataLine.back() << std::endl;
  } else {
    // Throw an error message if file failed to open
    throw std::runtime_error (mRobotName + ": Writing new line to " + mFileName + " failed!");
  }
}

//Write current simulation time and distance sensor readings to the appropriate .csv file
void CsvProcessor::writeData(double simTime, const std::array<double, DISTANCE_SENSOR_NUMBER> &dsValues) const {
  std::vector<double> dataLine; //A vector of doubles to store the data to be written
  dataLine.push_back(simTime);
  dataLine.insert(dataLine.end(), dsValues.begin(), dsValues.end());
  try {
    writeLineToCsv(dataLine); //Write the data to the appropriate .csv file
  } catch(const std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
  }
}

// Read lines of data from a .csv file and return a 2D double vector containing read data
// Code snippet taken and modified from Exercise 3
std::vector<std::vector<double> > CsvProcessor::readDataFromCsv() const {
  std::vector<std::vector<double> > dataMatrix; // 2D vector<double> to store lines of data read from the .csv file

  std::ifstream fIn(mFileName, std::ios::in);
  if (fIn) { // Check whether file is open
    std::string dataLine;

    while (std::getline(fIn, dataLine)) { // While the .csv file has more lines to be read
      std::stringstream sStream{ dataLine };

      std::vector<double> data;

      while (sStream.good()) { // While the line has more data elements to be read
        std::string dataString;
        std::getline(sStream, dataString, mDelim);
        data.push_back(std::stod(dataString)); // Push data element, converted from string to double, to the vector<double> data
      }
      dataMatrix.push_back(data); // Push complete vector<double> data to back of 2D vector<double> dataMatrix
    }
    fIn.close();
    std::cout << mRobotName << ": Reading data from " << mFileName << " succeeded!\n"; // .csv file has been fully read, notify user
  } else { // If file has failed to open, throw an error
    std::cout << mRobotName << ": Reading data from " << mFileName << " failed!\n";
  }
  return dataMatrix;
}

// For ROBOT_RED, process recorded data, finding maximum, minimum and average values of each column
// Based upon code written in Exercise 3
std::vector<std::vector<double> > CsvProcessor::rProcessData(const std::vector<std::vector<double> > &dataMatrix) const {
  std::vector<double> maxVals; // Stores maximum values of each data column
  std::vector<double> minVals; // Stores minimum values of each data column
  std::vector<double> valSums; // Stores sums of all values in each data column
  auto firstRow {*dataMatrix.begin()};
  for (auto col = firstRow.begin(); col != firstRow.end(); col++) { // Initial values for all data vectors should be first row of data
    maxVals.push_back(*col);
    minVals.push_back(*col);
    valSums.push_back(*col);
  }

  for (auto row = std::next(dataMatrix.begin()); row != dataMatrix.end(); row++) { // Iterate through rows and columns of data matrix, starting from the second row
    for (auto col = row->begin(); col != row->end(); col++) {
      int colIndex = col - row->begin(); // Get current column number as an index starting from 0
      if (*col > maxVals[colIndex]) { // Accumulate the maximum value in each column
        maxVals[colIndex] = *col;
      }
      if (*col < minVals[colIndex]) { // Accumulate the minimum value in each column
        minVals[colIndex] = *col;
      }
      valSums[colIndex] += *col; // Accumulate the sum of all values in each column
    }
  }
  
  std::vector<double> avgVals;
  int nRows {(int)dataMatrix.size()}; // Calculate number of data rows
  for (auto iter = valSums.begin(); iter != valSums.end(); iter++) { // Calculate average of each column from sum of each column divided by number of rows
    double avgVal = *iter / nRows;
    avgVals.push_back(avgVal); // Store calculated average in avgVals vector
  }
  
  // Store max, min and average values as rows in a single 2D double vector and return this
  std::vector<std::vector<double> > processedData;
  processedData.push_back(maxVals);
  processedData.push_back(minVals);
  processedData.push_back(avgVals);
  return processedData;
}

// Static function to compare the magnitude of the final element of two double vectors, used in bProcessData function
// Code based upon reference provided in Assignment 1 Description: https://www.geeksforgeeks.org/sorting-2dvector-in-c-set-1-by-row-and-column/
bool CsvProcessor::compCol(const std::vector<double> &v1, const std::vector<double> &v2) {
  return v1.back() < v2.back();
}

// For ROBOT_BLUE, process recorded data, sorting rows by their last column, and write processed data to a separate file
// Code snippet taken and modified from Exercise 3
void CsvProcessor::bProcessData(std::vector<std::vector<double> > &dataMatrix) const {
  std::sort(dataMatrix.begin(), dataMatrix.end(), compCol); // Sort rows by their last column (ascending order)
  std::string fName = mRobotName + "_sorted.csv";
  std::ofstream fout {fName, std::ios::out|std::ios::trunc};  // Open file in output, truncate mode (Will overwrite any existing contents)
  if (fout.is_open()) {  // Check whether the file was opened successfully
    for (auto row = dataMatrix.begin(); row != dataMatrix.end(); row++) { // Write each row of sorted values to the output file
      for (auto col = row->begin(); col != row->end(); col++) {
        fout << std::fixed << std::setprecision(3) << *col << mDelim; // Write each data value to three decimal-places in fixed-point notation, separated by commas
      }      
      fout << std::endl;
    }
    std::cout << mRobotName << ": Writing data to " << fName << " succeeded!\n"; // Notify user that data writing succeeded
  } else {
    throw std::runtime_error (mRobotName + ": Writing data to " + fName + " failed!\n"); // Throw appropriate error message if opening the file failed
  }
}

// Print the results of data processing from ROBOT_RED in the required format
// Code snippet taken and modified from Exercise 3
void CsvProcessor::rPrintData(const std::vector<std::vector<double> > &dataMatrix) const {
  const std::vector<std::string> vLabels {"maximum", "minimum", "average"}; // Strings describing the type of processed data printed to cout
  for (auto row = dataMatrix.begin(); row != dataMatrix.end(); row++) { // Iterate through rows and columns of data matrix
    int index = row - dataMatrix.begin(); // Get row index of current element from row iterator (to print correct label for the row e.g. maximum)
    std::cout << mRobotName << ": The " << vLabels[index] << " values are - "; // Print descriptive string at the start of eac data row
    for (auto col = row->begin(); col != row->end()-1; col++) {
      std::cout << std::fixed << std::setprecision(3) << *col << ", "; // Print processed data row by row, separated by commas and spaces
    }
    std::cout << std::fixed << std::setprecision(3) << *(row->end()-1) << std::endl; // Print final data value in row separately, without a comma
  }
}

// A superclass to allow for the control of the robot in any of the 6 possible modes
// Based upon code written in Exercise 4
class MyRobot : public webots::Robot {
public:
  MyRobot();                                                        // Constructor
  void run();                                                       // Main function of the superclass, controls all subclass operations to complete any tasks required
  int getTimeStep() const;                                          // Retrieve the basic time step of the simulation
private:
  std::string mRobotName;                                           // Name of the robot
  int mTimeStep;                                                    // Stores simulation time step
  double mSimTime;                                                  // Stores current simulation time
  std::array<webots::DistanceSensor*, DISTANCE_SENSOR_NUMBER> ds;   // Stores pointers to each distance sensor
  std::array<double, DISTANCE_SENSOR_NUMBER> dsValues;              // Stores current reading values of each distance sensor
  webots::Motor *leftMotor;                                         // Pointer to left motor object
  webots::Motor *rightMotor;                                        // Pointer to right motor object
  webots::Camera *camera;                                           // Pointer to camera object
  webots::Keyboard *keyboard;                                       // Pointer to keyboard object
  opMode mMode;                                                     // Stores current operating mode of robot
  manCommand mCommand;                                              // Stores current manual-control mode command
  TaskManager taskMan;                                              // An instance of the TaskManager class
  SpeedController spdCon;                                           // An instance of the SpeedController class
  CsvProcessor csvProc;                                             // An instance of the CsvProcessor class
  void setDSVals();                                                 // Update the stored distance sensor reading values
  bool findGreen() const;                                           // Determine whether the camera is viewing a green wall
  bool findBlue() const;                                            // Determine whether the camera is viewing a blue wall
  bool endReached() const;                                          // Determine whether the end of the track has been reach in shortest-time mode
};

// Constructor for the MyRobot class
// Code snippets taken and modified from Exercise 3/WeBots Tutorial page
MyRobot::MyRobot() {
  mRobotName = getName(); // Get and store the name of the robot

  mTimeStep = (int)getBasicTimeStep(); // Get and store the basic time step for the robot
  
  // Initialise the array of distance sensors on the robot
  std::array<std::string, DISTANCE_SENSOR_NUMBER> dsNames {
      "ds0", "ds1", "ds2", "ds3"
  };
  for (int i = 0; i < DISTANCE_SENSOR_NUMBER; ++i) {
      ds[i] = getDistanceSensor(dsNames[i]);
      ds[i]->enable(mTimeStep);
  }
    
  // Initialise the left and right motors of the robot
  leftMotor = getMotor("left wheel motor");
  rightMotor = getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // Initialise the camera on the robot
  camera = getCamera("camera");
  camera->enable(mTimeStep);
  
  // Initialise the keyboard object on the robot
  keyboard = getKeyboard();
  keyboard->enable(mTimeStep);

  // Initialise the operating mode and manual command settings to default values
  mMode = opMode::Start;
  mCommand = manCommand::Pause;

  // Initialise CsvProcessor class instance
  // (TaskManager and SpeedController class instances are implicitly initialised with their defined default constructors, which is desired)
  std::string fileName = mRobotName + ".csv";
  csvProc = CsvProcessor(mRobotName, fileName);
};

// The main function of the MyRobot class, that drives functionality of all subclasses and performs any required operation
void MyRobot::run() {
  setDSVals(); // Update distance sensor readings
  int key = keyboard->getKey(); //Get curent keyboard input
  mSimTime = getTime(); // Update stored simulation time
  taskMan.printOutput(mRobotName, mMode, mSimTime); // Print any relevant command options or info based on current mode/operation state
  std::array<double, MOTOR_NUMBER> speeds {0.0, 0.0}; // Initialise motor speeds array with zeroes at each iteration
  if (!taskMan.isTaskComplete()) { // Only continue operating if the selected task is not yet complete
    switch (mMode) {
      case opMode::Start: // Need this here to avoid getting a non-fatal compiler warning
      break;
      case opMode::Manual: // Manual control mode
        csvProc.writeData(mSimTime, dsValues); // Record data for current time
        mCommand = mRobotName == "ROBOT_RED" ? taskMan.readManCmdRed(mCommand, key) : taskMan.readManCmdBlue(mCommand, key); //Get correct user key input based on robot name
        speeds = spdCon.getMCModeSpds(mCommand); // Set robot motor speed values based upon user input
      break;
      case opMode::Follow: // Wall-Following mode
        if (findGreen()) { // If a green wall is detected, the robot has reached its destination, stop task and print appropriate user prompt
          taskMan.setTaskComplete(true);
          taskMan.setCmdsPrinted(false);
        } else { // Otherwise, get appropriate motor speed settings for current stage in task
          speeds = mRobotName == "ROBOT_RED" ? spdCon.getWFModeSpdsRed(dsValues) : spdCon.getWFModeSpdsBlue(dsValues);
        }
      break;
      case opMode::Short: // Shortest time mode
        if (endReached()) { // If the destination has been reached, stop task and print appropriate user prompt
          taskMan.setTaskComplete(true);
          taskMan.setCmdsPrinted(false);
          taskMan.printOutput(mRobotName, mMode, mSimTime);
        } else { // Otherwise, get appropriate motor speed settings for current stage in task
          speeds = spdCon.getSTModeSpds(mTimeStep, dsValues);
        }
      break;
      case opMode::Read: // Data-read mode
        if (mRobotName == "ROBOT_RED") { // For ROBOT_RED, process stored data and print to console output
          std::vector<std::vector<double> > dataMatrix = csvProc.readDataFromCsv();
          if (dataMatrix.size() > 0) {
            std::vector<std::vector<double> > pData = csvProc.rProcessData(dataMatrix);
            csvProc.rPrintData(pData);
          }
        } else { // For ROBOT_BLUE, process stored data and write to new file
          std::vector<std::vector<double> > dataMatrix = csvProc.readDataFromCsv();
          if (dataMatrix.size() > 0) {
            csvProc.bProcessData(dataMatrix);
          }
        }
        taskMan.setTaskComplete(true); // Task is complete, avoid printing further console output
      break;
    }

    // Set motor speeds to stored values, which have been modified above where appropriate
    leftMotor->setVelocity(speeds[0]);
    rightMotor->setVelocity(speeds[1]);

    // Read user input to select operating mode, if the robot is not already in an operating mode
    if (mMode == opMode::Start) {
      mMode = taskMan.readCommand(mMode, mRobotName, key);
    }
  }
}

// Retrieve and store current distance sensor readings
// Code snippet taken from Exercise 3/WeBots Tutorial page
void MyRobot::setDSVals() {
  for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++) {
    dsValues[i] = ds[i]->getValue();
  }
}

// Determine whether or not the camera is currently viewing a green object
// Code snippet taken and modified from Exercise 3/WeBots Tutorial page
bool MyRobot::findGreen() const {
  bool greenFound = false;
  // Get image from camera
  const unsigned char *image = camera->getImage();
  if (image) { // Image may be NULL if Robot.synchronization is FALSE
      int red {0};
      int green {0};
      int blue {0};

      int camWidth = camera->getWidth();
      int camHeight = camera->getHeight();

      for (int i = 0; i < camWidth; ++i) { // Iterate through pixels at the center of the image
          for (int j = 0; j < camHeight; ++j) {
              red += camera->imageGetRed(image, camWidth, i, j);
              blue += camera->imageGetBlue(image, camWidth, i, j);
              green += camera->imageGetGreen(image, camWidth, i, j);
          }
      }
      
      if ((green > COLOUR_THRESHOLD * red) && (green > COLOUR_THRESHOLD * blue)) { // If green pixel count greatly outweighs blue or red, green is detected
          greenFound = true;
      }
  }
  return greenFound;
}

// Determine whether or not the camera is currently viewing a blue object
// Code snippet taken and modified from Exercise 3/WeBots Tutorial page
bool MyRobot::findBlue() const {
  bool blueFound = false;
  // Get image from camera
  const unsigned char *image = camera->getImage();
  if (image) { // Image may be NULL if Robot.synchronization is FALSE
      int red {0};
      int green {0};
      int blue {0};

      int camWidth = camera->getWidth();
      int camHeight = camera->getHeight();

      for (int i = 0; i < camWidth; ++i) { // Iterate through pixels at the center of the image
          for (int j = 0; j < camHeight; ++j) {
              red += camera->imageGetRed(image, camWidth, i, j);
              blue += camera->imageGetBlue(image, camWidth, i, j);
              green += camera->imageGetGreen(image, camWidth, i, j);
          }
      }
      
      if ((blue > COLOUR_THRESHOLD * red) && (blue > COLOUR_THRESHOLD * green)) { //If blue pixel count greatly outweighs red or green, blue is detected
          blueFound = true;
      }
  }
  return blueFound;
}

// Determine whether the robot has reached its destination in shortest-time mode
// Must be close to a wall and camera must be viewing a green or blue wall
bool MyRobot::endReached() const {
  return (dsValues[1] > NEAR_OBS_THRESHOLD || dsValues[2] > NEAR_OBS_THRESHOLD) && (findBlue() || findGreen());
}

// Get the current timestep of the simulation
int MyRobot::getTimeStep() const {
  return mTimeStep;
}

// Main loop, runs the simulation until stopped
int main(int argc, char **argv) {
  MyRobot robot {}; // Initialise MyRobot superclass
  int timeStep = robot.getTimeStep(); // Retrieve the time step of the simulation

  while (robot.step(timeStep) != -1) { // Run the MyRobot::run() function each simulation step
    robot.run();
  };

  return 0;
}
