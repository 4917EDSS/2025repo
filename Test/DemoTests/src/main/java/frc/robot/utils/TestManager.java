// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.tests.RunTestsGrp;

/**
 * This class holds all of the test results and handles adding them to the Shuffleboard.
 */
public class TestManager {
  /**
   * An enumeration type that represents fail/warn/pass internally and associated integers for
   * use with the dashboard.
   */
  public enum Result {
    kFail(-1), kWarn(0), kPass(1);

    private int value;

    Result(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  /**
   * A simple class to represent the location of dashboard widgets
   */
  private class Coordinates {
    public int m_x = 0;
    public int m_y = 0;

    public Coordinates(int x, int y) {
      m_x = x;
      m_y = y;
    }
  }

  /**
   * A class to store the current status of individual tests as well as information about their
   * dashboard widgets.
   */
    public class Test {

      private final TestManager m_testManager;
      // member variables
      private int m_id;
      private String m_name;
      public Result m_result;
      private String m_status;
      private List<TestResult> m_resultsList;
      public String m_text;
      public GenericEntry m_resultDisplay;
      public GenericEntry m_textDisplay;
  
  
      // getters and setters
      public int getId() {
        return m_id;
      }
      public String getName() {
        return m_name;
      }
  
      public Result getResult() {
        return m_result;
      }
  
      public String getStatus() {
        return m_status;
      }
  
      // factory method for a test
      public Test(TestManager testManager, int id, String name){
        this.m_testManager = testManager;
        m_id = id;
        m_name = name;
        m_resultsList = new ArrayList<TestResult>();
      }
  
      public TestResult addResult(String measuredAttribute){
        TestResult testResult = new TestResult(measuredAttribute);
        return testResult;
      }
  
      public TestResult addResult(String measuredAttribute, double actualValue, double targetValue, double tolerance, double minimumValue) {
        // add the result to the result list
        TestResult testResult = new TestResult(measuredAttribute, actualValue, targetValue, tolerance, minimumValue);
        return testResult;
      }
  
      private void calculateResults() {
          // generate descriptive text for the result
          for (TestResult result : m_resultsList) {
            //calculate all the results
            result.setResult(m_testManager.determineResult(result.getActualValue(), 
                                result.getTargetValue(),
                                result.getFailTolerance(), 
                                result.getWarnTolerance()));
          }
      }
  
      public void updateStatus() {
        calculateResults();
        m_testManager.updateTestStatus(this);
      }
    
      public void interrupt(){
        m_status = "Test interrupted";
        m_result = Result.kFail;
        for (TestResult testResult : m_resultsList) {
          testResult.setResult(Result.kFail);
        }
        m_testManager.updateTestStatus(this);
      }
    }

  /*
   * TestResult is a class to store the parameters for a test
   * 
   *  - Measured attribute -> the name of the attribute like Amps or Position
   *  - Measured value -> the result of the test
   *  - Target value -> the expected value of the measured attribute
   *  - fail tolerance +/- -> If the absolute value is > that this the test will fail
   *  - warn tolerance +/- -> If the absolute value is > that this the test will warn
   * 
   */
  public class TestResult {

    //member variables
    private String m_measuredAttribute;
    private Double m_actualValue;
    private Double m_targetValue;
    private double m_failTolerance;
    private Double m_warnTolerance;
    private Result m_result;

    
    public Double getActualValue() {
        return m_actualValue;
    }
    
    public Double getTargetValue() {
        return m_targetValue;
    }
    
    public double getFailTolerance() {
        return m_failTolerance;
    }
    
    public Double getWarnTolerance() {
        return m_warnTolerance;
    }
    
    public Result getResult() {
        return m_result;
    }
    
    public void setResult(Result result) {
        m_result = result;
    }
    
    public TestResult(String measuredAttribute){
        m_measuredAttribute = measuredAttribute;
    }

    public TestResult(String measuredAttribute, Double actualValue, Double targetValue, Double toleranceFail, Double toleranceWarn){
        m_actualValue = actualValue;
        m_targetValue = targetValue;
        m_failTolerance = toleranceFail;
        m_warnTolerance = toleranceWarn;
    }

    public TestResult withActualValue(Double actualValue){
        m_actualValue = actualValue;
        return this;
    }

    public TestResult withTargetValue(Double targetValue){
        m_targetValue = targetValue;
        return this;
    }

    public TestResult withFailTolerance(Double failTolerance){
        m_failTolerance = failTolerance;
        return this;
    }

    public TestResult withWarnTolerance(Double warnTolerance){
        m_warnTolerance = warnTolerance;
        return this;
    }

    @Override
    public String toString(){
        StringBuilder sb = new StringBuilder();
        sb.append(m_measuredAttribute);
        sb.append(" (Target=");
        sb.append(m_targetValue);
        sb.append(" +/- ");
        sb.append(m_failTolerance);
        sb.append(")");
        return sb.toString();
    }
  }
  
  // Member variables
  private final ShuffleboardTab m_boardTab;
  private GenericEntry m_overallStatusDisplay;
  private ArrayList<Test> m_tests;
  private int m_nextTestStatusIdx = 0;
  private Coordinates m_nextTestCoordinates;

  /**
   * Constructor. Need to also call setTestCommand before this class will be usable.
   * 
   * @param testsCommand Command that runs all of the tests
   */
  public TestManager() {
    // Grab the Tests tab on the Shuffleboard.  It should already have been created.
    m_boardTab = Shuffleboard.getTab(Constants.Tests.kTabName);

    // Setup the location where test widgets can be added (below the overall indicators that are
    // set up later)
    m_tests = new ArrayList<Test>();
    m_nextTestCoordinates = new Coordinates(1, 0);

    System.out.println("Creating Test Manager");
  }

  /**
   * Sets the test command. This needs to be done after the constructor because the test command
   * takes this class as a parameter so it can request new tests.
   * 
   * @param testsCommand Command to use to start all tests
   */
  public void setTestCommand(RunTestsGrp testsCommand) {
    // Add the overall-tests-result indicator and the button to start the tests
    m_overallStatusDisplay = m_boardTab.add("Overall Status", Result.kFail.getValue())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1, "max", 1, "center", -1, "show text", false, "num tick marks", 2))
        .withSize(1, 1)
        .withPosition(0, 0)
        .getEntry();
    m_boardTab.add(testsCommand)
        .withSize(1, 1)
        .withPosition(1, 0);

    System.out.println("Setting test command");
  }

  /**
   * Creates a new test entry on the dashboard. Includes test result and status text.
   * 
   * @param name Descriptive name for the test
   * @return Test ID. Used to update the test information in the future
   */
  public int registerNewTest(String name) {
    // Add a new test entry at the next open index.  Use the index as the test ID.
    m_tests.add(m_nextTestStatusIdx, new Test(this, m_nextTestStatusIdx, name));

    // Now add the test result and status test widgets to the dashboard
    Coordinates testLocation = getNextPosition();

    System.out.println("Registering New Test: " + testLocation.m_x + " " + testLocation.m_y);

    Test newTest = m_tests.get(m_nextTestStatusIdx);
    newTest.m_result = Result.kFail;
    newTest.m_resultDisplay = m_boardTab.add(name + " Result", newTest.m_result.getValue())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1, "max", 1, "center", -1, "show text", false, "num tick marks", 2))
        .withSize(1, 1)
        .withPosition(testLocation.m_y, testLocation.m_x)
        .getEntry();
    newTest.m_text = "Not run";
    newTest.m_textDisplay = m_boardTab.add(name + " Status", newTest.m_text)
        .withSize(1, 1)
        .withPosition(testLocation.m_y + 1, testLocation.m_x)
        .getEntry();

    // Return the array index (i.e. the ID) of the test and then increment it for the next one
    return m_nextTestStatusIdx++;
  }

  /**
   * Reset all of the test statuses to "fail" to start a new test run
   */
  public void resetTestStatuses() {
    // Reset all individual tests
    for(int id = 0; id < m_tests.size(); id++) {
      updateTestStatus(id, Result.kFail, "Not Run");
    }

    // Reset overall status
    updateOverallStatus();
  }

  /**
   * Method that triggers the update of the overall status on the dashboard
   */
  public void updateOverallStatus() {
    Result overallResult = Result.kPass; // Assume a pass unless we find test that didn't

    for(Test test : m_tests) {
      if(test.m_result == Result.kFail) {
        overallResult = Result.kFail;
        break;
      } else if(test.m_result == Result.kWarn) {
        overallResult = Result.kWarn;
      }
    }

    // Update the overall result on the Shuffleboard
    m_overallStatusDisplay.setInteger(overallResult.getValue());
  }

  /**
   * Update the result and text for a single test
   * 
   * @param id ID of the test as returned from registerNewTest()
   * @param result Current result of the test
   * @param status Status text describing the result
   */
  public void updateTestStatus(int id, Result result, String status) {
    Test currentTest = m_tests.get(id);

    currentTest.m_result = result;
    currentTest.m_text = status;
    currentTest.m_resultDisplay.setInteger(result.getValue());
    currentTest.m_textDisplay.setString(status);
  }

  public void updateTestStatus(Test test) {
    updateTestStatus(test.getId(), test.getResult(),test.getStatus());
  }

  /**
   * Utility function that checks if the values provided should yield a pass, warn or fail
   * 
   * @param actualValue The value produced by the test
   * @param targetValue The value that the test should have produced
   * @param tolerance The tolerance + or - around the targetValue that is still considered a pass
   * @param minimumValue The minimum value that the value can be to produce a warn
   * @return Pass, warn or fail determination
   */
  public Result determineResult(double actualValue, double targetValue, double tolerance, double minimumValue) {
    Result calculatedResult = Result.kFail; // Assume a fail unless proven otherwise

    if(Math.abs(actualValue - targetValue) < tolerance) {
      calculatedResult = Result.kPass;
    } else if(actualValue > minimumValue) {
      calculatedResult = Result.kWarn;
    }

    return calculatedResult;
  }

  /**
   * 
   * @param name A descriptive name for the test
   * @return a Test to used to calculate results
   */
  public Test getTest(String name){
    Test t = new Test(this, registerNewTest(name), name);
    // registered now send it back
    return t;
  }

  /**
   * Get the next free location
   * 
   * @return Coordinates of the next free dashboard location
   */
  private Coordinates getNextPosition() {
    Coordinates nextPosition = new Coordinates(m_nextTestCoordinates.m_x, m_nextTestCoordinates.m_y);

    // Move to the next position after this one.
    m_nextTestCoordinates.m_x++;

    // Make sure we aren't falling off the bottom of the dashboard
    if(m_nextTestCoordinates.m_x > Constants.Tests.kDashboardRows) {
      m_nextTestCoordinates.m_x = 0;
      m_nextTestCoordinates.m_y += 2;
    }

    // TODO: Make sure we're not falling off the right side of the dashboard.  If so, create a new tab

    return nextPosition;
  }
}