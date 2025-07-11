#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <deque>
#include <numeric>
#include <vector>
#include <algorithm>
#include <cstdlib>

using namespace std;

const float MAX_TORQUE = 145.0f;
const float FIXED_POWER_WATTS = 121.0f * 735.5f; // ~88995 W
const float WHEEL_RADIUS = 0.4064f;              // 16 inches in meters
const float dt = 0.1f;

float clamp(float val, float minVal, float maxVal)
{
    return max(minVal, min(val, maxVal));
}

float computeAverage(const vector<float> &values)
{
    if (values.empty())
        return 0.0f;
    float sum = accumulate(values.begin(), values.end(), 0.0f);
    return sum / values.size();
}

class DrivingSimulation
{
private:
    string terrain, style;
    float speedLimit = 0.0f;
    float maxRPM, maxThrottle, rampTime;
    int noiseRange;
    float speedFactor, baseEfficiencyRPM, mileageEfficiencyFactor;
    float fuel = 0.4f;
    float fuelUsed = 0.0f;
    float totalDistance = 0.0f;
    float totalTime = 0.0f;
    float lastMileage = 0.0f;
    float rpm = 0.0f;
    float instMileage = 0.0f;

    vector<float> allRPMs, allSpeeds, allTorques, allMileages;
    deque<float> mileageHistory;
    const int windowSize = 10;

public:
    void getUserInput()
    {
        cout << "🌍 Enter terrain type (hill/plain/downward): ";
        cin >> terrain;

        if (terrain == "hill")
            speedLimit = 100.0f;
        else if (terrain == "plain")
            speedLimit = 150.0f;
        else if (terrain == "downward")
            speedLimit = 70.0f;
        else
        {
            cerr << "❌ Invalid terrain. Exiting.\n";
            exit(1);
        }

        cout << "🧍 Enter driving style (conservative/moderate/aggressive): ";
        cin >> style;
    }

    void configureStyle()
    {
        if (style == "conservative")
        {
            maxRPM = 2500.0f;
            maxThrottle = 2000.0f;
            rampTime = 18.0f;
            noiseRange = 200;
            speedFactor = 0.7f;
            baseEfficiencyRPM = 2000.0f;
            mileageEfficiencyFactor = 20.0f;
        }
        else if (style == "moderate")
        {
            maxRPM = 3000.0f;
            maxThrottle = 2500.0f;
            rampTime = 20.0f;
            noiseRange = 200;
            speedFactor = 0.85f;
            baseEfficiencyRPM = 2200.0f;
            mileageEfficiencyFactor = 17.0f;
        }
        else if (style == "aggressive")
        {
            maxRPM = 6000.0f;
            maxThrottle = 4800.0f;
            rampTime = 12.0f;
            noiseRange = 1000;
            speedFactor = 1.08f;
            baseEfficiencyRPM = 2600.0f;
            mileageEfficiencyFactor = 15.0f;
        }
        else
        {
            cout << "Invalid style. Using 'moderate'...\n";
            maxRPM = 3000.0f;
            maxThrottle = 2500.0f;
            rampTime = 20.0f;
            noiseRange = 200;
            speedFactor = 0.85f;
        }
    }

    void simulate()
    {
        cout << fixed << setprecision(2);
        cout << "\n🚗 Simulation Start on " << terrain << " terrain as a " << style << " driver...\n\n";

        for (float t = 0.0f; fuel - fuelUsed > 0.01f; t += dt)
        {
            if (terrain == "hill" && style == "conservative")
            {
                maxRPM = 2800.0f;
                maxThrottle = 2500.0f;
                rampTime = 20.0f;
                noiseRange = 200;
                speedFactor = 0.85f;
                baseEfficiencyRPM = 2200.0f;
                mileageEfficiencyFactor = 18.0f;
            }

            if (terrain == "downward" && style == "aggressive")
            {
                maxRPM = 4000.0f;
                maxThrottle = 3000.0f;
                rampTime = 12.0f;
                noiseRange = 850;
                speedFactor = 1.00f;
                baseEfficiencyRPM = 2600.0f;
                mileageEfficiencyFactor = 15.0f;
            }

            float throttle = maxThrottle * (1.0f - exp(-t / rampTime));
            float noise = rand() % (3 * noiseRange) - noiseRange;
            rpm = clamp(throttle + noise, 0.0f, maxRPM);

            float kmph = clamp((rpm / maxRPM) * speedLimit * speedFactor, 0.0f, speedLimit);
            float speed = kmph / 3.6f;

            float torque = (rpm >= 5000.0f) ? MAX_TORQUE : (rpm / 5000.0f) * MAX_TORQUE;

            float distance = speed * dt;
            totalDistance += distance;

            float efficiencyDrop = pow(rpm / baseEfficiencyRPM, 1.2f);
            float dynamicMileage = clamp((1.0f / efficiencyDrop) * mileageEfficiencyFactor, 5.0f, 25.0f);

            float instFuelUsed = (distance / 1000.0f) / dynamicMileage;
            fuelUsed += instFuelUsed;
            float fuelLeft = fuel - fuelUsed;

            if (totalDistance < 0.001f)
                instMileage = 0.0f;
            else if (instFuelUsed > 0.0001f)
                instMileage = (distance / 1000.0f) / instFuelUsed;
            else
                instMileage = lastMileage;

            lastMileage = instMileage;
            float rangeLeft = fuelLeft * instMileage;

            mileageHistory.push_back(instMileage);
            if (mileageHistory.size() > windowSize)
                mileageHistory.pop_front();

            allRPMs.push_back(rpm);
            allSpeeds.push_back(kmph);
            allTorques.push_back(torque);
            allMileages.push_back(instMileage);
            totalTime = t;

            cout << "[t=" << t << "s] "
                 << "RPM=" << rpm << ", "
                 << "Speed=" << kmph << " km/h, "
                 << "Torque=" << torque << " Nm, "
                 << "Mileage=" << instMileage << " km/l, "
                 << "FuelLeft=" << fuelLeft << " L, "
                 << "RangeLeft=" << rangeLeft << " km\n";

            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }

    void printReport()
    {
        float avgSpeed = computeAverage(allSpeeds);
        float avgRPM = computeAverage(allRPMs);
        float avgTorque = computeAverage(allTorques);
        float overallMileage = totalDistance / 1000.0f / fuelUsed;
        float maxMileage = *max_element(allMileages.begin(), allMileages.end());
        float minMileage = *min_element(allMileages.begin(), allMileages.end());

        int behaviorScore = 0;
        if (overallMileage >= 15.0f)
            behaviorScore++;
        if (avgSpeed < 60.0f)
            behaviorScore++;
        if (avgRPM < 2500.0f)
            behaviorScore++;
        if (avgTorque < 80.0f)
            behaviorScore++;

        string behavior;
        if (behaviorScore >= 3)
            behavior = "Eco-friendly Driver";
        else if (behaviorScore == 2)
            behavior = "Moderate Driver";
        else
            behavior = "Aggressive Driver";

        cout << "\n🚗 FINAL DRIVING REPORT 🚗\n";
        cout << "Terrain:            " << terrain << "\n";
        cout << "Style:              " << style << "\n";
        cout << "Total Time:         " << totalTime << " seconds\n";
        cout << "Total Distance:     " << totalDistance / 1000.0f << " km\n";
        cout << "Fuel Consumed:      " << fuelUsed << " L\n";
        cout << "Average Speed:      " << avgSpeed << " km/h\n";
        cout << "Average RPM:        " << avgRPM << "\n";
        cout << "Average Torque:     " << avgTorque << " Nm\n";
        cout << "Overall Mileage:    " << overallMileage << " km/l\n";
        cout << "Max Mileage:        " << maxMileage << " km/l\n";
        cout << "Min Mileage:        " << minMileage << " km/l\n";
        cout << "Driver Behavior:    " << behavior << "\n";
        cout << "------------------------------------------------\n";
    }
};

int main()
{
    DrivingSimulation sim;
    sim.getUserInput();
    sim.configureStyle();
    sim.simulate();
    sim.printReport();
    return 0;
}
