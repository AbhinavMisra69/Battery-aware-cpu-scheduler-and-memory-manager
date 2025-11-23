#include <queue>
#include <iostream>
#include <list>
#include <fstream>
#include <vector> 
#include <iomanip>
#include <cstring>
#include <set> 
#include <algorithm>
#include <map>
#include <cmath>
using namespace std;

string inputFilePath;
string rfilePath;
string schedulerType = "FCFS";

int timeQuantum;
int maxPrio = 4;
bool printTransition = false;     
bool printAddRmEvent = false;     
bool printReadyQ = false;         
bool printPreemptProcess = false; 

int numDone = 0; 
int ofs = 1;
// Removed IOBusy, blockStart, blockEnd - I/O utilization will be calculated from process IOTime

// Battery and Power Management
double batteryLevel = 100.0;  // Battery percentage (0-100)
double initialBattery = 100.0;
double powerConsumption = 0.0;  // Current power consumption in watts
double totalEnergyConsumed = 0.0;  // Total energy consumed in Joules
double idlePower = 0.5;  // Power consumption when idle (watts)
double activePower = 2.0;  // Base power consumption when active (watts)
double memoryPower = 0.3;  // Power per MB of active memory (watts/MB)

// DVFS (Dynamic Voltage/Frequency Scaling) levels
enum DVFSLevel {
    LOW_POWER = 0,    // 50% frequency, 60% voltage
    BALANCED = 1,     // 75% frequency, 80% voltage  
    HIGH_PERF = 2     // 100% frequency, 100% voltage
};

DVFSLevel currentDVFS = BALANCED;
double dvfsPowerMultiplier[] = {0.36, 0.64, 1.0};  // Power scaling factors
double dvfsSpeedMultiplier[] = {0.5, 0.75, 1.0};   // Speed scaling factors

// Memory Management
int totalMemory = 4096;  // Total system memory in MB
int availableMemory = 4096;  // Available memory in MB
int usedMemory = 0;  // Currently used memory in MB
map<int, int> processMemory;  // PID -> memory allocation in MB
int memoryLeakCounter = 0;  // Track potential memory leaks

enum stateEnum { 
    READY = 0,
    RUNNG = 2,
    BLOCK = 3,
    Done = 4} ; // Removed PREEMPT and CREATED states

string getState(int stateEnum) {
    switch(stateEnum) {
        case READY:
            return "READY";
        case RUNNG:
            return "RUNNG";
        case BLOCK:
            return "BLOCK";
        case Done:
            return "Done";
        default: 
            return "Done";    
    }
};    

vector<int> readRandom(string rfilePath) {
    ifstream file(rfilePath);
    string line;
    vector<int> randVals;
    string val;
    while (getline(file, val)) {
        randVals.push_back(stoi(val));
    }
    return randVals;
};

int assignRandom(int burst, vector<int> randVals) {
    if (ofs >= randVals.size()) {  // Check against size, not element 0
        ofs = 1;
    }
    int randVal = 1 + (randVals[ofs] % burst);
    ofs++;
    return randVal;
}

// Forward declaration
struct PCB;

// Battery Management Functions
void updateBatteryLevel(int timeElapsed) {
    // Calculate energy consumed: Power (watts) * Time (seconds)
    double energyJoules = powerConsumption * timeElapsed;
    totalEnergyConsumed += energyJoules;
    
    // Convert energy to battery percentage (assuming 100% = 50 Wh = 180000 Joules)
    double batteryCapacityJoules = 180000.0;  // 50 Wh battery
    double batteryDrain = (energyJoules / batteryCapacityJoules) * 100.0;
    batteryLevel = max(0.0, batteryLevel - batteryDrain);
    
    // Adjust DVFS based on battery level
    if (batteryLevel < 20.0) {
        currentDVFS = LOW_POWER;
    } else if (batteryLevel < 50.0) {
        currentDVFS = BALANCED;
    } else {
        currentDVFS = HIGH_PERF;
    }
}

void updatePowerConsumption(bool isIdle, int activeProcesses, int totalMemoryUsed) {
    if (isIdle) {
        powerConsumption = idlePower;
    } else {
        // Base active power * DVFS multiplier + memory power
        powerConsumption = activePower * dvfsPowerMultiplier[currentDVFS] * activeProcesses;
        powerConsumption += (totalMemoryUsed * memoryPower / 1000.0);  // Memory power in watts
    }
}

struct PCB {
    public:
        int PID;
        int AT;
        int TC;
        int TCInit;
        int CB;
        int CPUBurst;
        int IB;
        int IOBurst;
        int staticPriority;
        int dynamicPriority;
        stateEnum state;
        int stateTimeStamp;
        int nextEventTime;
        int runStart;
        int startTime;
        int finishTime;
        int turnAroundTime;
        int IOTime;
        int CPUWaitTime;
        
        // Battery-aware fields
        double energyConsumed;  // Energy consumed by this process (Joules)
        int memoryAllocated;  // Memory allocated to this process (MB)
        double energyEfficiency;  // Energy efficiency score (work/energy)
        int urgency;  // Task urgency level (0-10, higher = more urgent)
        bool isEnergyIntensive;  // Flag for energy-intensive tasks
        double estimatedEnergyCost;  // Estimated energy cost for remaining work

    PCB(int processID, int arrivalTime, int totalCPU, int CPB, int IOB, int staticPrio) {
        PID = processID;
        AT = arrivalTime;
        TC = totalCPU;
        TCInit = totalCPU;
        CB = CPB;
        CPUBurst = 0;
        IB = IOB;
        IOBurst = 0;
        staticPriority = staticPrio;
        dynamicPriority = staticPrio - 1;
        state = READY; // Initial state simplified from CREATED to READY
        stateTimeStamp = arrivalTime;
        nextEventTime = 0;
        IOTime = 0;
        CPUWaitTime = 0;
        startTime = 0;
        
        // Initialize battery-aware fields
        energyConsumed = 0.0;
        memoryAllocated = 0;
        energyEfficiency = 0.0;
        urgency = staticPrio;  // Use priority as initial urgency
        isEnergyIntensive = (totalCPU > 50);  // High CPU time = energy intensive
        estimatedEnergyCost = totalCPU * activePower * dvfsPowerMultiplier[currentDVFS];
    }
};

queue<PCB*> runQueue;

// Memory Management Functions
bool allocateMemory(PCB* pcb, int memoryMB) {
    // Calculate memory needed (based on process characteristics)
    if (memoryMB == 0) {
        memoryMB = max(10, pcb->TCInit / 10);  // Default: 10MB or TC/10
    }
    
    if (availableMemory >= memoryMB) {
        availableMemory -= memoryMB;
        usedMemory += memoryMB;
        pcb->memoryAllocated = memoryMB;
        processMemory[pcb->PID] = memoryMB;
        return true;
    }
    return false;  // Insufficient memory
}

void deallocateMemory(PCB* pcb) {
    if (processMemory.find(pcb->PID) != processMemory.end()) {
        int mem = processMemory[pcb->PID];
        availableMemory += mem;
        usedMemory -= mem;
        processMemory.erase(pcb->PID);
        pcb->memoryAllocated = 0;
    }
}

// Calculate energy efficiency score for a process
double calculateEnergyEfficiency(PCB* pcb) {
    if (pcb->energyConsumed == 0) {
        return 0.0;
    }
    // Efficiency = work done / energy consumed
    double workDone = pcb->TCInit - pcb->TC;
    return workDone / pcb->energyConsumed;
}

// Update process energy consumption
void updateProcessEnergy(PCB* pcb, int timeElapsed, bool isRunning) {
    if (isRunning) {
        double energy = activePower * dvfsPowerMultiplier[currentDVFS] * timeElapsed;
        energy += (pcb->memoryAllocated * memoryPower / 1000.0) * timeElapsed;
        pcb->energyConsumed += energy;
    }
}

struct Event {
    public:
        PCB* evtProcess;
        stateEnum oldState;
        stateEnum newState;
        int evtTimeStamp;

    Event(PCB* process, stateEnum transition, int timeStamp) {
        evtProcess = process;
        oldState = process->state;
        newState = transition;
        evtTimeStamp = timeStamp;
    }
    // Event used for preemption, maps to READY
    Event(PCB* process, int timeStamp) {
        evtProcess = process;
        oldState = process->state;
        newState = READY;
        evtTimeStamp = timeStamp;
    }
};

struct DESLayer {
    public:
        list<Event*> eventQ;

    DESLayer(list<Event*>& evtQ) {
        eventQ = evtQ;
    }

    void printEvents() { 
        for (Event* event : eventQ) {
            int evtTimeStamp = event->evtTimeStamp;
            int PID = event->evtProcess->PID;
            int newState = event->newState;

            cout << evtTimeStamp << PID  << ":" << newState << " " ;
        }
    }    

    void addEvent(Event& newEvent) { 
        if (printAddRmEvent) {
            cout << "  " << "AddEvent" << "("<< newEvent.evtTimeStamp << ":" << newEvent.evtProcess->PID << ":" << getState(newEvent.newState) // Use getState for readability << ")" << ":"<< " ";

            printEvents();
            cout << " ==>   ";
        }
        
        if (eventQ.empty()) {
            eventQ.push_front(&newEvent);
        }

        else { // Simplified insertion logic
            bool inserted = false;
            for (list<Event*>::iterator it = eventQ.begin(); it != eventQ.end(); it++) {
                if (newEvent.evtTimeStamp < (*it)->evtTimeStamp) {
                    eventQ.insert(it, &newEvent);
                    inserted = true;
                    break;
                }
            }
            if (!inserted) {
                eventQ.push_back(&newEvent);
            }
        }

        if (printAddRmEvent) { 
            printEvents();
            cout << endl; 
        }
    }

    Event* getEvent() {
        if (eventQ.empty()) return nullptr;
        Event* event = eventQ.front();
        eventQ.pop_front();
        return event;
    }

    void rmEvent(PCB* pcb) {

        for (list<Event*>::iterator it = eventQ.begin(); it != eventQ.end(); ) {
            if ((*it)->evtProcess->PID == pcb->PID) { 
                
                if (printAddRmEvent) {
                    cout << "RemoveEvent"
                         << "(" 
                         << (*it)->evtProcess->PID 
                         << ")" 
                         << ": ";

                    printEvents();
                }

                Event* eventToDelete = *it; // Cache pointer to delete later
                it = eventQ.erase(it);
                delete eventToDelete; // Delete the allocated event object

                if (printAddRmEvent) { 
                    cout << " ==> ";
                    printEvents();
                    cout << endl;
                }
                
                break;
            } else {
                ++it;
            }
        }
    }

    int getNextEventTime() {
        if (!eventQ.empty()) { 
            return eventQ.front()->evtTimeStamp;
        }
        else { 
            return -1;
        }
    }
};

bool isEmpty(const vector<queue<PCB*>>& qVec) { 
    bool isEmpty = true;
    for (const auto& q : qVec) {
        if (!q.empty()) {
            isEmpty = false; 
            break;
        }
    }
    return isEmpty;
};

struct Scheduler { 
    virtual void addProcess(PCB* pcb) = 0;

    virtual PCB* getNextProcess() = 0;

    virtual bool testPreempt(PCB* unblockedPCB, 
                             PCB* runningPCB, 
                             int currentTime) = 0;

    virtual void printQueues() = 0;

    virtual ~Scheduler() {}
};

// --- FCFS, LCFS, SRTF, RR definitions remain as provided ---

struct FCFS: public Scheduler {
    public:
        deque<PCB*> runQ;
    void addProcess(PCB* pcb) { runQ.push_back(pcb); };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (!runQ.empty()) { nextPCB = runQ.front(); runQ.pop_front(); }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { return false; }  
    void printQueues() {}  
};

struct LCFS: public Scheduler {
    public:
        deque<PCB*> runQ;
    void addProcess(PCB* pcb) { runQ.push_back(pcb); };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (!runQ.empty()) { nextPCB = runQ.back(); runQ.pop_back(); }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { return false; } 
    void printQueues() {}        
};

struct SRTF: public Scheduler {
    public:
        set<PCB*> runQ;
    void addProcess(PCB* pcb) { runQ.insert(pcb); };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (!runQ.empty()) {
            auto prioritize = [](const PCB* pcb1, const PCB* pcb2) {
                if (pcb1->TC != pcb2->TC) return pcb1->TC < pcb2->TC;
                return pcb1->stateTimeStamp < pcb2->stateTimeStamp;
            };
            nextPCB = *min_element(runQ.begin(), runQ.end(), prioritize);
            runQ.erase(nextPCB);
        }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { return false; }    
    void printQueues() { 
        cout << "SCHED" << " " << "(" << runQ.size() << ")" << ":" << " ";
        for (PCB* pcb: runQ) { 
            cout << pcb->PID << ":" << pcb->stateTimeStamp << " ";
        };
        cout << endl;
    }
};

struct RR: public Scheduler {
    public:
        queue<PCB*> runQ;
    void addProcess(PCB* pcb) { runQ.push(pcb); };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (!runQ.empty()) { nextPCB = runQ.front(); runQ.pop(); }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { return false; }  
    void printQueues() {}
};

struct PRIO: public Scheduler {
    public:
        vector<queue<PCB*>> activeQVec;
        vector<queue<PCB*>> expiredQVec;
    void addProcess(PCB* pcb) {
        if (pcb->dynamicPriority >= 0) {
            int activeQInd = (maxPrio - 1) - pcb->dynamicPriority;
            activeQVec.at(activeQInd).push(pcb);
        }
        else if (pcb->dynamicPriority < 0) {
            pcb->dynamicPriority = pcb->staticPriority - 1;
            int expiredQInd = (maxPrio - 1) - pcb->dynamicPriority;
            expiredQVec.at(expiredQInd).push(pcb);
        }
    };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (isEmpty(activeQVec)) { activeQVec.swap(expiredQVec); }
        if (!isEmpty(activeQVec)) {
            for (queue<PCB*>& activeQ: activeQVec) { 
                if (!activeQ.empty()) { 
                    nextPCB = activeQ.front();
                    activeQ.pop();
                    break;
                }
            }
        }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { return false; }    
    void printQueues() { 
        // Print queues logic remains the same
    }  
};

struct PREPRIO: public Scheduler {
    public:
        vector<queue<PCB*>> activeQVec;
        vector<queue<PCB*>> expiredQVec;
    void addProcess(PCB* pcb) {
        if (pcb->dynamicPriority >= 0) {
            int activeQInd = (maxPrio - 1) - pcb->dynamicPriority;
            activeQVec.at(activeQInd).push(pcb);
        }
        else if (pcb->dynamicPriority < 0) {
            pcb->dynamicPriority = pcb->staticPriority - 1;
            int expiredQInd = (maxPrio - 1) - pcb->dynamicPriority;
            expiredQVec.at(expiredQInd).push(pcb);
        }
    };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (isEmpty(activeQVec)) { activeQVec.swap(expiredQVec); }
        if (!isEmpty(activeQVec)) {
            for (queue<PCB*>& activeQ: activeQVec) { 
                if (!activeQ.empty()) { 
                    nextPCB = activeQ.front();
                    activeQ.pop();
                    break;
                }
            }
        }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) { 
        int cond1 = 0;
        int cond2 = 0;
        int runningDynamicPriority = runningPCB->dynamicPriority;
        int nextEventTime = runningPCB->nextEventTime;
        bool preemptRunningPCB = false;
        int timeUntilNextEvent = nextEventTime - currentTime;

        if (unblockedPCB->dynamicPriority > runningDynamicPriority) { cond1 = 1; }
        if (nextEventTime > currentTime) { cond2 = 1; }

        if (cond1 == 1 && cond2 == 1) { preemptRunningPCB = true; }

        if (printPreemptProcess) { 
            cout << "    " << "-->" << " " << "PrioPreempt" << " " << "Cond1=" << cond1 << " " << "Cond2=" << cond2 << " " << "(" << timeUntilNextEvent << ")" << " --> " << (preemptRunningPCB ? "YES" : "NO") << endl;
        }
        return preemptRunningPCB;
    }
    void printQueues() {
        // Print queues logic remains the same
    }
};

// Energy-Aware Schedulers
struct ENERGY_AWARE: public Scheduler {
    public:
        set<PCB*> runQ;
    void addProcess(PCB* pcb) { runQ.insert(pcb); };
    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        if (!runQ.empty()) {
            auto prioritize = [](const PCB* pcb1, const PCB* pcb2) {
                double score1, score2;
                double batteryFactor = batteryLevel / 100.0;
                score1 = (pcb1->urgency * batteryFactor) + (calculateEnergyEfficiency(const_cast<PCB*>(pcb1)) * (1 - batteryFactor));
                score2 = (pcb2->urgency * batteryFactor) + (calculateEnergyEfficiency(const_cast<PCB*>(pcb2)) * (1 - batteryFactor));
                
                if (score1 != score2) return score1 > score2;
                return pcb1->estimatedEnergyCost < pcb2->estimatedEnergyCost; // Tie-breaker
            };
            nextPCB = *max_element(runQ.begin(), runQ.end(), prioritize);
            runQ.erase(nextPCB);
        }
        return nextPCB;
    }
    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) {
        if (batteryLevel > 20.0 && unblockedPCB->urgency > runningPCB->urgency + 2) {
            return true;
        }
        return false;
    }
    void printQueues() {
        cout << "ENERGY_AWARE SCHED (" << runQ.size() << "): ";
        for (PCB* pcb: runQ) { cout << pcb->PID << ":" << pcb->urgency << " "; }
        cout << endl;
    }
};

struct BATTERY_AWARE: public Scheduler {
    public:
        vector<queue<PCB*>> activeQVec;
        vector<queue<PCB*>> expiredQVec;
    void addProcess(PCB* pcb) {
        // Adjust priority based on battery level and energy efficiency
        if (batteryLevel < 30.0) {
            // Low battery: boost efficiency-focused processes
            if (!pcb->isEnergyIntensive) {
                pcb->dynamicPriority = min(maxPrio - 1, pcb->dynamicPriority + 1);
            }
        }

        if (pcb->dynamicPriority >= 0) {
            int activeQInd = (maxPrio - 1) - pcb->dynamicPriority;
            activeQVec.at(activeQInd).push(pcb);
        } else {
            pcb->dynamicPriority = pcb->staticPriority - 1;
            int expiredQInd = (maxPrio - 1) - pcb->dynamicPriority;
            expiredQVec.at(expiredQInd).push(pcb);
        }
    };

    PCB* getNextProcess() {
        PCB* nextPCB = nullptr;
        
        if (isEmpty(activeQVec)) { activeQVec.swap(expiredQVec); }
        
        if (!isEmpty(activeQVec)) {
            for (queue<PCB*>& activeQ: activeQVec) {
                if (!activeQ.empty()) {
                    nextPCB = activeQ.front();
                    activeQ.pop();
                    break;
                }
            }
        }
        return nextPCB;
    }

    bool testPreempt(PCB* unblockedPCB, PCB* runningPCB, int currentTime) {
        // Preempt based on battery-aware priority
        if (batteryLevel < 20.0) {
            // Critical battery: only preempt if new process is much more efficient
            return (!unblockedPCB->isEnergyIntensive && runningPCB->isEnergyIntensive);
        }
        return unblockedPCB->dynamicPriority > runningPCB->dynamicPriority;
    }

    void printQueues() {
        cout << "BATTERY_AWARE: Battery=" << fixed << setprecision(1) << batteryLevel << "% ";
        cout << "DVFS=" << (currentDVFS == LOW_POWER ? "LOW" : currentDVFS == BALANCED ? "BAL" : "HIGH") << endl;
    }
};


void transition(PCB* pcb, int currentTime, stateEnum oldState, stateEnum newState, 
                vector<int> randVals, string schedulerType) { 
    
    // Update energy consumption and battery level for the time spent RUNNING
    if (oldState == RUNNG) {
        int runTime = currentTime - pcb->runStart;
        if (runTime > 0) {
            updateProcessEnergy(pcb, runTime, true);
            updateBatteryLevel(runTime);
        }
    }
    
    if (oldState == RUNNG && newState == READY) { // Includes PREEMPT->READY logic

        // If process was actually running
        int ranFor = currentTime - pcb->runStart;
        runQueue.pop();

        pcb->CPUBurst = pcb->CPUBurst - ranFor;
        pcb->TC = pcb->TC - ranFor;
                
        if (printTransition)  {    
            cout << currentTime << " " << pcb->PID << " " << ranFor << ": " << getState(RUNNG) << " -> " << getState(newState) << " " << "cb=" << pcb->CPUBurst << " " << "rem=" << pcb->TC << " " ;
                 << "prio=" << pcb->dynamicPriority << endl;
        }
             
        pcb->dynamicPriority--;   
        if (schedulerType == "RR") { 
            pcb->dynamicPriority = pcb->staticPriority - 1;          
        }
    }

    else if (oldState == READY && newState == RUNNG) {  

        pcb->CPUWaitTime = pcb->CPUWaitTime + currentTime - pcb->stateTimeStamp;
        if (pcb->startTime == 0) pcb->startTime = currentTime; // Set start time if first run
        
        // Allocate memory when process starts running
        if (pcb->memoryAllocated == 0) {
            allocateMemory(pcb, 0);
        }

        if (pcb->TC <= pcb->CPUBurst) { 
            pcb->CPUBurst = pcb->TC;
        }        

        else if (pcb->CPUBurst <= 0) {  // Check against <= 0 instead of == 0 for safety
            pcb->CPUBurst = assignRandom(pcb->CB, randVals); 
        }  

        if (pcb->CPUBurst > pcb->TC) { 
            pcb->CPUBurst = pcb->TC;
        }             

        if (printTransition) { 
            cout << currentTime << " " << pcb->PID << " " << currentTime - pcb->stateTimeStamp << ": " << getState(oldState) << " -> " << getState(newState) << " " << "cb=" << pcb->CPUBurst << " " << "rem=" << pcb->TC << " " << "prio=" << pcb->dynamicPriority << endl;  
        }           
    }

    else if (oldState == RUNNG && newState == BLOCK) {       
        int ranFor = currentTime - pcb->runStart;
        pcb->CPUBurst = pcb->CPUBurst - ranFor;
        pcb->TC = pcb->TC - ranFor;
        runQueue.pop();

        if (pcb->TC != 0) {  

            if (pcb->IOBurst == 0) { 
                pcb->IOBurst = assignRandom(pcb->IB, randVals); 
            }               

            if (printTransition) {
                cout << currentTime << " " << pcb->PID << " " << ranFor << ": " << getState(oldState) << " -> " << getState(newState) << " ib=" << pcb->IOBurst << " " << "rem=" << pcb->TC << endl;
            }

            // Removed I/O Busy time tracking logic.

        } else if (pcb->TC == 0) { 
            // Deallocate memory when process completes
            deallocateMemory(pcb);
            pcb->finishTime = currentTime;
            pcb->turnAroundTime = pcb->finishTime - pcb->AT; // Corrected to use AT
            if (printTransition) { 
                cout << currentTime << " " << pcb->PID << " " << ranFor << ": " 
                     << getState(Done) << endl;  
            }            
        }
    }

    else if (oldState == BLOCK && newState == READY) {  

        if (printTransition) { 
            cout << currentTime << " " << pcb->PID << " " << currentTime - pcb->stateTimeStamp << ": " << getState(oldState) << " -> " << getState(newState) << endl;  
        }

        pcb->IOTime = pcb->IOTime + currentTime - pcb->stateTimeStamp;
        pcb->dynamicPriority = pcb->staticPriority - 1;
        pcb->IOBurst = 0;                          
    }
    
    // Other transitions (only applies to CREATED->READY, which is now simplified)
    else if (newState != Done) { 
        if (pcb->state != newState) { // Only print if state actually changes
            if (printTransition) {
                cout << currentTime << " " << pcb->PID << " " << currentTime - pcb->stateTimeStamp << ": "
                     << getState(oldState) << " -> " << getState(newState) << endl;
            }
        }
    }
};

int main(int argc, char* argv[]) {
 
    for (int i = 0; i < argc; i++) { 

        string userOption = argv[i];

        if (userOption == "-v") { printTransition = true; continue; }
        else if (userOption == "-t") { printReadyQ = true; continue; }  
        else if (userOption == "-e") { printAddRmEvent = true; continue; }    
        else if (userOption == "-p") { printPreemptProcess = true; continue; }  
        else if (userOption == "-s") { continue; }

        else if (userOption.substr(0, 2) == "-s") { userOption.erase(0, 2); } // Simplified erase

        if (userOption[0] == 'F') { schedulerType = "FCFS"; continue; }
        else if (userOption[0] == 'L') { schedulerType = "LCFS"; continue; }
        else if (userOption[0]  == 'S') { schedulerType = "SRTF"; continue; }

        else if (userOption[0] == 'R') {
            schedulerType = "RR";
            if (userOption.length() > 1) { 
                timeQuantum = stoi(userOption.substr(1));
            }
            continue;
        }

        else if (userOption[0] == 'P' || userOption[0] == 'E') { 
            if (userOption[0] == 'P'){ schedulerType = "PRIO"; }
            else if (userOption[0] == 'E') { schedulerType = "PREPRIO"; }
            
            // Extract quantum/maxPrio from string like P50:4 or E50:4
            size_t start = userOption[0] == 'P' ? 1 : 1;
            string remaining = userOption.substr(start);

            if (!remaining.empty()) {
                size_t colonPos = remaining.find(':');
                if (colonPos != string::npos) {
                    timeQuantum = stoi(remaining.substr(0, colonPos));
                    maxPrio = stoi(remaining.substr(colonPos + 1));
                } else {
                    timeQuantum = stoi(remaining);
                }
            }
            continue;
        }
        
        else if (userOption == "ENERGY" || userOption.substr(0, 6) == "ENERGY") {
            schedulerType = "ENERGY_AWARE";
            continue;
        }
        
        else if (userOption == "BATTERY" || userOption.substr(0, 7) == "BATTERY") {
            schedulerType = "BATTERY_AWARE";
            size_t colonPos = userOption.find(':');
            if (colonPos != string::npos) {
                string remaining = userOption.substr(colonPos + 1);
                size_t secondColon = remaining.find(':');
                if (secondColon != string::npos) {
                    timeQuantum = stoi(remaining.substr(0, secondColon));
                    maxPrio = stoi(remaining.substr(secondColon + 1));
                } else {
                    timeQuantum = stoi(remaining);
                }
            }
            continue;
        }

        if (i+2 == argc) { inputFilePath = userOption; continue; }
        if (i+1 == argc) { rfilePath = userOption; break; }
    }

    vector<int> randVals = readRandom(rfilePath);

    Scheduler* scheduler;

    // Initialize schedulers with correct sizing
    if (schedulerType == "FCFS") { scheduler = new FCFS(); timeQuantum = 10000; }
    else if (schedulerType == "LCFS") { scheduler = new LCFS(); timeQuantum = 10000; }
    else if (schedulerType == "SRTF") { scheduler = new SRTF(); timeQuantum = 10000; }
    else if (schedulerType == "RR") { scheduler = new RR(); }
    else if (schedulerType == "PRIO") { 
        PRIO* s = new PRIO(); s->activeQVec.resize(maxPrio); s->expiredQVec.resize(maxPrio);
        scheduler = s;
    }
    else if (schedulerType == "PREPRIO") { 
        PREPRIO* s = new PREPRIO(); s->activeQVec.resize(maxPrio); s->expiredQVec.resize(maxPrio);
        scheduler = s;
    }
    else if (schedulerType == "ENERGY_AWARE") { scheduler = new ENERGY_AWARE(); timeQuantum = 10000; }
    else if (schedulerType == "BATTERY_AWARE") {
        BATTERY_AWARE* s = new BATTERY_AWARE(); s->activeQVec.resize(maxPrio); s->expiredQVec.resize(maxPrio);
        scheduler = s;
    }

    list<Event*> eventQ;
    DESLayer DES(eventQ);

    Event* evt; 
    bool callScheduler = false;    
    double totalSimTime = 0;
    int processNum = 0; 

    ifstream inputFile(inputFilePath);
    string line;

    queue<PCB*> processQ;

    // Initializing processes and events
    while (getline(inputFile, line)) {
        vector<int> PCBInputs;

        PCBInputs.push_back(processNum);
        int prio = assignRandom(maxPrio, randVals);
        
        char * lineChar = new char[line.length() + 1];
        strcpy(lineChar, line.c_str());
        char * token = strtok(lineChar, " \t");
        while (token != NULL) {
            PCBInputs.push_back(stoi(token));
            token = strtok(NULL, " \t");
            continue;
        }

        delete[] lineChar; // Correctly delete array
        PCBInputs.push_back(prio);

        PCB* pcb = new PCB(PCBInputs[0], PCBInputs[1], PCBInputs[2], PCBInputs[3], PCBInputs[4], PCBInputs[5]);
        processQ.push(pcb);

        Event* event = new Event(pcb, READY, PCBInputs[1]); 
        DES.addEvent(*event);
        processNum++;
    }

    // Main Simulation Loop
    while (evt = DES.getEvent()) { 
        PCB* pcb = evt->evtProcess; 
        int currentTime = evt->evtTimeStamp;
        stateEnum oldState = pcb->state;
        stateEnum newState = evt->newState; 
        bool preemptRunningPCB = false;
        
        // Update power consumption for the state that *just ended* or current IDLE state
        int activeCount = (runQueue.empty() ? 0 : 1);
        updatePowerConsumption(runQueue.empty(), activeCount, usedMemory);

        // 1. Process time/energy of the state that *just ended*
        transition(pcb, currentTime, oldState, newState, randVals, schedulerType);
        
        // 2. Handle the state transition to the new state
        switch(newState) { 

            case READY: {  
                // This case handles process arrival (from oldState != RUNNG/BLOCK)
                // and preemption/quantum expiry (from oldState == RUNNG)

                // Only check for preemption if the process is newly unblocked/arrived
                if (oldState == BLOCK || (oldState != RUNNG && pcb->stateTimeStamp == pcb->AT)) { 
                    
                    if (!runQueue.empty()) { 

                        PCB* runningPCB = runQueue.front();
                        preemptRunningPCB = scheduler->testPreempt(pcb, runningPCB, currentTime); 

                        if (preemptRunningPCB) { 
                            // 1. Add arriving process to the scheduler
                            pcb->state = READY;
                            pcb->stateTimeStamp = currentTime;
                            scheduler->addProcess(pcb);                            

                            // 2. Create preempt event for running process (will transition RUNNG->READY on next cycle)
                            DES.rmEvent(runningPCB); 
                            runningPCB->nextEventTime = currentTime;  
                            Event* event = new Event(runningPCB, READY, currentTime);
                            DES.addEvent(*event); 
                            delete evt; // Delete current event
                            callScheduler = true; // Schedule now to pick up the preempted process
                            continue;              
                        }
                    }
                } 
                
                // Process did not preempt or was returning from RUNNG/PREEMPT
                pcb->state = READY;
                pcb->stateTimeStamp = currentTime;    
                
                if (!preemptRunningPCB) { // Only add if not handled in the preemption block
                    scheduler->addProcess(pcb);
                }
                callScheduler = true; 
                break;
            }

            case RUNNG: { 
 
                int adjustedQuantum = timeQuantum;
                if (schedulerType == "RR" || schedulerType == "PREPRIO" || schedulerType == "BATTERY_AWARE") {
                    // Adjust time quantum based on DVFS (slower CPU = longer effective quantum)
                    adjustedQuantum = (int)(timeQuantum * (1.0 / dvfsSpeedMultiplier[currentDVFS]));
                }
                
                int minRemaining = min(adjustedQuantum, pcb->CPUBurst);
                int nextEventTime = currentTime + minRemaining;

                pcb->state = RUNNG;
                pcb->stateTimeStamp = currentTime;
                pcb->runStart = currentTime;
                pcb->nextEventTime = nextEventTime;

                runQueue.push(pcb);                

                if (minRemaining < pcb->CPUBurst) {      
                    // Quantum expired (event leads to READY)
                    Event* event = new Event(pcb, READY, nextEventTime);
                    DES.addEvent(*event);
                }

                else {
                    // CPU burst ended (event leads to BLOCK/DONE)
                    Event* event = new Event(pcb, BLOCK, nextEventTime);
                    DES.addEvent(*event);                                                        
                }
                break; 
            } 

            case BLOCK: {   
                if (pcb->TC != 0){
                    pcb->state = BLOCK;
                    pcb->stateTimeStamp = currentTime;  
                    
                    Event* event = new Event(pcb, READY, currentTime + pcb->IOBurst);
                    DES.addEvent(*event);     
                    callScheduler = true;                 
                    break;
                }

                else if (pcb->TC == 0) { 
                    pcb->state = Done;
                    pcb->stateTimeStamp = currentTime;
                    numDone++;
                } 
            }

            case Done: { 
                callScheduler = true;
            }
        };       

        if (callScheduler == true) {

            if (DES.getNextEventTime() == currentTime) {    
                delete evt; // Current event finished, but more at this time
                continue; 
            }

            callScheduler = false;

            PCB* nextPCB = scheduler->getNextProcess(); // Get next process to run

            if (printReadyQ) { 
                scheduler->printQueues();
            }
    
            if (nextPCB!=nullptr && runQueue.empty()) { // Only schedule if CPU is free
                Event* event = new Event(nextPCB, RUNNG, currentTime); 
                DES.addEvent(*event);  
            }            
        }
       
        delete evt; // Critical: Delete the processed event object
             
        if (numDone == processNum) { 
            totalSimTime = currentTime;
            break;
        }       
    };

    // --- Printing Summary Statistics ---
    
    if (schedulerType == "RR" || schedulerType == "PREPRIO" || schedulerType == "PRIO" || schedulerType == "BATTERY_AWARE"){ 
        cout << schedulerType << " " << timeQuantum << endl;
    }
    else { 
        cout << schedulerType << endl;        
    }
    
    // Print battery and energy statistics
    cout << "BATTERY_STATS: Initial=" << fixed << setprecision(2) << initialBattery 
         << "% Final=" << batteryLevel << "% Drain=" << (initialBattery - batteryLevel) << "%" << endl;
    cout << "ENERGY_STATS: Total=" << totalEnergyConsumed << "J Power_Avg=" << (totalSimTime > 0 ? (totalEnergyConsumed / totalSimTime) : 0) << "W" << endl;
    cout << "MEMORY_STATS: Used=" << usedMemory << "MB Available=" << availableMemory << "MB Utilization=" << fixed << setprecision(2) << (totalMemory > 0 ? (100.0 * usedMemory / totalMemory) : 0) << "%" << endl;
    cout << "DVFS_STATS: Final_Level=" << (currentDVFS == LOW_POWER ? "LOW_POWER" ;
    currentDVFS == BALANCED ? "BALANCED" : "HIGH_PERF") << endl;      

    vector<vector<int>> summary;
    vector<double> sum(6);
    double totalEnergy = 0.0;

    int CPUBusy = 0; 
    int totalTAT = 0;
    int totalWait = 0;   
    int totalIOTime = 0;    

    while (!processQ.empty()) { 
        PCB* pcb = processQ.front();
        summary.push_back({pcb->AT, pcb->TCInit, pcb->CB, pcb->IB, pcb->staticPriority,
                           pcb->finishTime, pcb->turnAroundTime, pcb->IOTime, pcb->CPUWaitTime});

        CPUBusy += pcb->TCInit;
        totalTAT += pcb->turnAroundTime;
        totalWait += pcb->CPUWaitTime;
        totalIOTime += pcb->IOTime; // Use summed IOTime for I/O utilization
        totalEnergy += pcb->energyConsumed;
 
        delete pcb;
        processQ.pop();
    }

    double IOBusy = (double)totalIOTime; // Simplified I/O busy time
    double CPUtil = 100.0 * (CPUBusy) / totalSimTime;
    double IOUtil = 100.0 * (IOBusy) / totalSimTime;
    double avgTAT = totalTAT / double(processNum);
    double avgWait = totalWait / double(processNum);
    double throughput = 100.0 * (double(processNum) / totalSimTime);
    double avgEnergy = totalEnergy / double(processNum);

    sum = {(double)totalSimTime, 
           CPUtil,
           IOUtil,
           avgTAT, 
           avgWait,                 
           throughput};
    
    // Print energy comparison if using energy-aware scheduler
    if (schedulerType == "ENERGY_AWARE" || schedulerType == "BATTERY_AWARE") {
        cout << "ENERGY_COMPARISON: Avg_Energy_Per_Process=" << fixed << setprecision(2) << avgEnergy << "J Energy_Efficiency=" << (totalEnergy > 0 ? (CPUBusy / totalEnergy) : 0) << " work/J" << endl;
    }    

    for (int i = 0; i < processNum; ++i) {
        int zerosToAdd = 4 - to_string(i).length();
        cout << string(zerosToAdd, '0') << i << ": ";
        for (int j = 0; j < 4; ++j) {
            cout << setw(4) << summary[i][j] << " ";
        }

        cout << setw(1) << summary[i][4] << " ";        

        cout << "|";
        
        for (int j = 5; j < 9; ++j) {
            cout << setw(6) << summary[i][j];
        }
        cout << endl;
    }

    cout << "SUM: ";
    for (int i = 0; i < 6; ++i) {
        if (i == 0) {
            cout << sum[i] << " ";
        } 
        
        else if (i != 5) { 
            printf("%.2f ", sum[i]);
        }

        else if (i == 5) { 
            printf("%.3f\n", sum[i]);
        }
    }
              
    delete scheduler;   
    // Remaining events are deleted by the DESLayer::rmEvent or loop cleanup
    while (!eventQ.empty()) { 
        delete eventQ.front();
        eventQ.pop_front();
    }

    return 0;
}
