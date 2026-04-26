#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/msg.h>
#include <pthread.h>
#include <limits.h>
#include <unistd.h>
#include <math.h>

#define MAX_TRUCKS 250
#define TRUCK_MAX_CAP 20
#define MAX_NEW_REQUESTS 50
#define MAX_TOTAL_PACKAGES 5000

// Package Request Structure
typedef struct PackageRequest {
    int packageId;
    int pickup_x;
    int pickup_y;
    int dropoff_x;
    int dropoff_y;
    int arrival_turn;
    int expiry_turn;
} PackageRequest;

// Shared Memory Structure
typedef struct MainSharedMemory {
    char authStrings[MAX_TRUCKS][TRUCK_MAX_CAP + 1];
    char truckMovementInstructions[MAX_TRUCKS];
    int pickUpCommands[MAX_TRUCKS];
    int dropOffCommands[MAX_TRUCKS];
    int truckPositions[MAX_TRUCKS][2];
    int truckPackageCount[MAX_TRUCKS];
    int truckTurnsInToll[MAX_TRUCKS];
    PackageRequest newPackageRequests[MAX_NEW_REQUESTS];
    int packageLocations[MAX_TOTAL_PACKAGES][2];
} MainSharedMemory;

// Message Structures
typedef struct TurnChangeResponse {
    long mtype;
    int turnNumber;
    int newPackageRequestCount;
    int errorOccured;
    int finished;
} TurnChangeResponse;

typedef struct TurnReadyRequest {
    long mtype;
} TurnReadyRequest;

typedef struct SolverRequest {
    long mtype;
    int truckNumber;
    char authStringGuess[TRUCK_MAX_CAP + 1];
} SolverRequest;

typedef struct SolverResponse {
    long mtype;
    int guessIsCorrect;
} SolverResponse;

// Global Variables
int N, D, S, T, B;
key_t shmKey, mainMsgKey;
key_t solverKeys[MAX_TRUCKS];
int shmId, mainMsgId;
int solverMsgIds[MAX_TRUCKS];
MainSharedMemory *shmPtr;
int currentTurn = 0;

// Package tracking
typedef struct Package {
    int id;
    int pickup_x, pickup_y;
    int dropoff_x, dropoff_y;
    int arrival_turn;
    int expiry_turn;
    int status; // 0=pending, 1=pickedup, 2=delivered
    int assignedTruck; // -1 if not assigned
    int onTruck; // -1 if not on truck, else truck number
} Package;

Package packages[MAX_TOTAL_PACKAGES];
int packageExists[MAX_TOTAL_PACKAGES] = {0};

// Truck tracking
typedef struct Truck {
    int x, y;
    int packages[TRUCK_MAX_CAP];
    int packageCount;
    int turnsInToll;
} Truck;

Truck trucks[MAX_TRUCKS];

// Toll booth tracking
typedef struct TollBooth {
    int x, y;
    int waitTime;
} TollBooth;

TollBooth tollBooths[MAX_TRUCKS * MAX_TRUCKS];
int tollBoothCount = 0;
int tollMap[500][500]; // stores wait time, 0 if no toll

// A* Pathfinding structures
typedef struct AStarNode {
    int x, y;
    int g, h, f;
    int parent_x, parent_y;
} AStarNode;

typedef struct PriorityQueue {
    AStarNode nodes[100000];
    int size;
} PriorityQueue;

void pq_init(PriorityQueue *pq) {
    pq->size = 0;
}

void pq_push(PriorityQueue *pq, AStarNode node) {
    int i = pq->size++;
    pq->nodes[i] = node;
    
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[i].f >= pq->nodes[parent].f) break;
        
        AStarNode temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[parent];
        pq->nodes[parent] = temp;
        i = parent;
    }
}

AStarNode pq_pop(PriorityQueue *pq) {
    AStarNode result = pq->nodes[0];
    pq->nodes[0] = pq->nodes[--pq->size];
    
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < pq->size && pq->nodes[left].f < pq->nodes[smallest].f)
            smallest = left;
        if (right < pq->size && pq->nodes[right].f < pq->nodes[smallest].f)
            smallest = right;
        
        if (smallest == i) break;
        
        AStarNode temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[smallest];
        pq->nodes[smallest] = temp;
        i = smallest;
    }
    
    return result;
}

int manhattan_distance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// A* pathfinding with toll avoidance
char get_next_move(int start_x, int start_y, int goal_x, int goal_y) {
    if (start_x == goal_x && start_y == goal_y) return 's';
    
    static int visited[500][500];
    static int cost[500][500];
    
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            visited[i][j] = 0;
            cost[i][j] = INT_MAX;
        }
    }
    
    PriorityQueue pq;
    pq_init(&pq);
    
    AStarNode start;
    start.x = start_x;
    start.y = start_y;
    start.g = 0;
    start.h = manhattan_distance(start_x, start_y, goal_x, goal_y);
    start.f = start.g + start.h;
    start.parent_x = -1;
    start.parent_y = -1;
    
    pq_push(&pq, start);
    cost[start_x][start_y] = 0;
    
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};
    char moves[] = {'l', 'r', 'u', 'd'};
    
    while (pq.size > 0) {
        AStarNode current = pq_pop(&pq);
        
        if (visited[current.x][current.y]) continue;
        visited[current.x][current.y] = 1;
        
        if (current.x == goal_x && current.y == goal_y) {
            // Backtrack to find first move
            int bx = current.x, by = current.y;
            int px = current.parent_x, py = current.parent_y;
            
            while (px != start_x || py != start_y) {
                // Find parent of parent
                for (int i = 0; i < N; i++) {
                    for (int j = 0; j < N; j++) {
                        if (visited[i][j]) {
                            // Check if this could be parent
                            for (int d = 0; d < 4; d++) {
                                int nx = i + dx[d];
                                int ny = j + dy[d];
                                if (nx == px && ny == py) {
                                    if (cost[i][j] == cost[px][py] - 1 - tollMap[px][py] * 5) {
                                        bx = px; by = py;
                                        px = i; py = j;
                                        goto next_iter;
                                    }
                                }
                            }
                        }
                    }
                }
                next_iter:;
            }
            
            // Now px,py is start and bx,by is next cell
            for (int d = 0; d < 4; d++) {
                if (start_x + dx[d] == bx && start_y + dy[d] == by) {
                    return moves[d];
                }
            }
        }
        
        for (int i = 0; i < 4; i++) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];
            
            if (nx < 0 || nx >= N || ny < 0 || ny >= N) continue;
            if (visited[nx][ny]) continue;
            
            int newCost = current.g + 1 + tollMap[nx][ny] * 5; // Penalize tolls heavily
            
            if (newCost < cost[nx][ny]) {
                cost[nx][ny] = newCost;
                
                AStarNode next;
                next.x = nx;
                next.y = ny;
                next.g = newCost;
                next.h = manhattan_distance(nx, ny, goal_x, goal_y);
                next.f = next.g + next.h;
                next.parent_x = current.x;
                next.parent_y = current.y;
                
                pq_push(&pq, next);
            }
        }
    }
    
    // Fallback: move towards goal
    if (goal_x < start_x) return 'l';
    if (goal_x > start_x) return 'r';
    if (goal_y < start_y) return 'u';
    if (goal_y > start_y) return 'd';
    return 's';
}

// Thread structure for auth string guessing
typedef struct AuthThreadData {
    int truckId;
    int solverIdx;
    int length;
    char *result;
    int *found;
    pthread_mutex_t *mutex;
} AuthThreadData;

void generate_guess(char *guess, int length, int index) {
    char chars[] = {'u', 'd', 'l', 'r'};
    for (int i = 0; i < length; i++) {
        guess[length - 1 - i] = chars[index % 4];
        index /= 4;
    }
    guess[length] = '\0';
}

void *auth_guess_thread(void *arg) {
    AuthThreadData *data = (AuthThreadData *)arg;

    // ... (unchanged setup code for totalCombos, threadId, start, end, req, resp) ...
    int totalCombos = 1;
    for (int i = 0; i < data->length; i++) totalCombos *= 4;
    int threadId = data->solverIdx;
    int start = (totalCombos * threadId) / S;
    int end = (totalCombos * (threadId + 1)) / S;
    SolverRequest req;
    SolverResponse resp;
    req.mtype = 2;
    req.truckNumber = data->truckId;
    msgsnd(solverMsgIds[data->solverIdx], &req, sizeof(SolverRequest) - sizeof(long), 0);
    // ...

    for (int i = start; i < end; i++) {
        pthread_mutex_lock(data->mutex);
        if (*data->found) {
            pthread_mutex_unlock(data->mutex);
            break;
        }
        pthread_mutex_unlock(data->mutex);

        char full_guess[TRUCK_MAX_CAP + 1];
        
        // --- ROBUST PATCH START ---
        // 1. Clear the entire buffer to guarantee null termination and clean padding
        memset(full_guess, 0, sizeof(full_guess));
        
        // 2. Generate the correct prefix of length L (data->length)
        // This should write L characters and ideally null-terminate it at index L
        generate_guess(full_guess, data->length, i); 
        
        // 3. Explicitly null-terminate after the prefix, just in case generate_guess doesn't
        full_guess[data->length] = '\0';

        // 4. Pad the guess to TRUCK_MAX_CAP (20) with 's' (stay)
        for(int j = data->length; j < TRUCK_MAX_CAP; j++) {
            full_guess[j] = 's'; 
        }
        full_guess[TRUCK_MAX_CAP] = '\0'; // Final null termination (redundant but safe)

        // 5. Send the full padded string to the solver
        req.mtype = 3;
        strcpy(req.authStringGuess, full_guess);

        msgsnd(solverMsgIds[data->solverIdx], &req, sizeof(SolverRequest) - sizeof(long), 0);
        msgrcv(solverMsgIds[data->solverIdx], &resp, sizeof(SolverResponse) - sizeof(long), 4, 0);

        if (resp.guessIsCorrect) {
            pthread_mutex_lock(data->mutex);
            if (!*data->found) {
                // 6. Write the full padded string to the shared memory result
                strcpy(data->result, full_guess);
                *data->found = 1;
            }
            pthread_mutex_unlock(data->mutex);
            break;
        }
        // --- ROBUST PATCH END ---
    }

    return NULL;
}
// -----------------------------------------------------------------------------
void find_auth_string(int truckId, char *result) {
    // This function is fine. It correctly determines the significant length L.
    int length = trucks[truckId].packageCount;

    if (length == 0) {
        result[0] = '\0';
        return;
    }

    pthread_t threads[MAX_TRUCKS];
    AuthThreadData threadData[MAX_TRUCKS];
    int found = 0;
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

    for (int i = 0; i < S; i++) {
        threadData[i].truckId = truckId;
        threadData[i].solverIdx = i;
        threadData[i].length = length; // Correctly passing the significant length L
        threadData[i].result = result;
        threadData[i].found = &found;
        threadData[i].mutex = &mutex;

        pthread_create(&threads[i], NULL, auth_guess_thread, &threadData[i]);
    }

    for (int i = 0; i < S; i++) {
        pthread_join(threads[i], NULL);
    }

    pthread_mutex_destroy(&mutex);
}

void init_system() {
    // Read input
    FILE *fp = fopen("input.txt", "r");
    fscanf(fp, "%d %d %d %d %d", &N, &D, &S, &T, &B);
    fscanf(fp, "%d %d", &shmKey, &mainMsgKey);
    
    for (int i = 0; i < S; i++) {
        fscanf(fp, "%d", &solverKeys[i]);
    }
    fclose(fp);
    
    // Connect to shared memory
    shmId = shmget(shmKey, sizeof(MainSharedMemory), 0666);
    shmPtr = (MainSharedMemory *)shmat(shmId, NULL, 0);
    
    // Connect to message queues
    mainMsgId = msgget(mainMsgKey, 0666);
    for (int i = 0; i < S; i++) {
        solverMsgIds[i] = msgget(solverKeys[i], 0666);
    }
    
    // Initialize trucks
    for (int i = 0; i < D; i++) {
        trucks[i].x = 0;
        trucks[i].y = 0;
        trucks[i].packageCount = 0;
        trucks[i].turnsInToll = 0;
    }
    
    // Initialize toll map
    memset(tollMap, 0, sizeof(tollMap));
}

void update_state(TurnChangeResponse *resp) {
    currentTurn = resp->turnNumber;
    
    // CRITICAL: Always sync truck state from shared memory (authoritative source)
    for (int i = 0; i < D; i++) {
        trucks[i].x = shmPtr->truckPositions[i][0];
        trucks[i].y = shmPtr->truckPositions[i][1];
        trucks[i].packageCount = shmPtr->truckPackageCount[i]; // Use helper's count
        trucks[i].turnsInToll = shmPtr->truckTurnsInToll[i];
        
        // Detect toll booths
        if (shmPtr->truckTurnsInToll[i] > 0 && tollMap[trucks[i].x][trucks[i].y] == 0) {
            tollMap[trucks[i].x][trucks[i].y] = shmPtr->truckTurnsInToll[i];
        }
    }
    
    // Update package states based on locations
    for (int p = 0; p < MAX_TOTAL_PACKAGES; p++) {
        if (!packageExists[p]) continue;
        
        int loc_x = shmPtr->packageLocations[p][0];
        int loc_y = shmPtr->packageLocations[p][1];
        
        if (loc_x == -1 && loc_y == -1) {
            // Package is either on a truck or delivered
            if (packages[p].status == 1) {
                // Still on truck - find which one
                packages[p].onTruck = -1;
                for (int t = 0; t < D; t++) {
                    if (trucks[t].x == packages[p].dropoff_x && 
                        trucks[t].y == packages[p].dropoff_y &&
                        packages[p].status == 1) {
                        // Could be delivered, check next turn
                    }
                }
            }
        } else {
            // Package at a location
            if (packages[p].status == 0) {
                // Waiting for pickup
                packages[p].onTruck = -1;
            }
        }
    }
    
    // Rebuild truck package arrays based on what we know
    for (int t = 0; t < D; t++) {
        int count = 0;
        for (int p = 0; p < MAX_TOTAL_PACKAGES; p++) {
            if (packageExists[p] && packages[p].onTruck == t && packages[p].status == 1) {
                trucks[t].packages[count++] = p;
            }
        }
    }
    
    // Add new packages
    for (int i = 0; i < resp->newPackageRequestCount; i++) {
        PackageRequest *req = &shmPtr->newPackageRequests[i];
        packages[req->packageId].id = req->packageId;
        packages[req->packageId].pickup_x = req->pickup_x;
        packages[req->packageId].pickup_y = req->pickup_y;
        packages[req->packageId].dropoff_x = req->dropoff_x;
        packages[req->packageId].dropoff_y = req->dropoff_y;
        packages[req->packageId].arrival_turn = req->arrival_turn;
        packages[req->packageId].expiry_turn = req->expiry_turn;
        packages[req->packageId].status = 0;
        packages[req->packageId].assignedTruck = -1;
        packages[req->packageId].onTruck = -1;
        packageExists[req->packageId] = 1;
    }
}

void assign_packages() {
    // Simple greedy assignment: assign nearest available package to each free truck
    for (int t = 0; t < D; t++) {
        if (trucks[t].packageCount >= TRUCK_MAX_CAP) continue;
        if (trucks[t].turnsInToll > 0) continue;
        
        int bestPkg = -1;
        int bestDist = INT_MAX;
        int bestUrgency = INT_MAX;
        
        for (int p = 0; p < MAX_TOTAL_PACKAGES; p++) {
            if (!packageExists[p]) continue;
            if (packages[p].status != 0) continue;
            if (packages[p].assignedTruck != -1 && packages[p].assignedTruck != t) continue;
            
            int dist = manhattan_distance(trucks[t].x, trucks[t].y, 
                                        packages[p].pickup_x, packages[p].pickup_y);
            int urgency = packages[p].expiry_turn - currentTurn;
            
            if (urgency < bestUrgency || (urgency == bestUrgency && dist < bestDist)) {
                bestPkg = p;
                bestDist = dist;
                bestUrgency = urgency;
            }
        }
        
        if (bestPkg != -1) {
            packages[bestPkg].assignedTruck = t;
        }
    }
}

void plan_moves() {
    // Initialize commands
    for (int i = 0; i < D; i++) {
        shmPtr->truckMovementInstructions[i] = 's';
        shmPtr->pickUpCommands[i] = -1;
        shmPtr->dropOffCommands[i] = -1;
        shmPtr->authStrings[i][0] = '\0';
    }
    
    for (int t = 0; t < D; t++) {
        if (trucks[t].turnsInToll > 0) continue;
        
        // Check if we can drop a package
        for (int i = 0; i < trucks[t].packageCount; i++) {
            int pkgId = trucks[t].packages[i];
            if (trucks[t].x == packages[pkgId].dropoff_x && 
                trucks[t].y == packages[pkgId].dropoff_y) {
                shmPtr->dropOffCommands[t] = pkgId;
                packages[pkgId].status = 2;
                packages[pkgId].onTruck = -1;
                break;
            }
        }
        
        // Check if we can pick up a package
        if (trucks[t].packageCount < TRUCK_MAX_CAP && shmPtr->dropOffCommands[t] == -1) {
            for (int p = 0; p < MAX_TOTAL_PACKAGES; p++) {
                if (!packageExists[p]) continue;
                if (packages[p].assignedTruck != t) continue;
                if (packages[p].status != 0) continue;
                
                if (trucks[t].x == packages[p].pickup_x && 
                    trucks[t].y == packages[p].pickup_y) {
                    shmPtr->pickUpCommands[t] = p;
                    packages[p].status = 1;
                    packages[p].onTruck = t;
                    trucks[t].packages[trucks[t].packageCount++] = p;
                    break;
                }
            }
        }
        
        // Determine next move
        int target_x, target_y;
        if (trucks[t].packageCount > 0) {
            // Go to dropoff of first package
            int pkgId = trucks[t].packages[0];
            target_x = packages[pkgId].dropoff_x;
            target_y = packages[pkgId].dropoff_y;
        } else {
            // Go to pickup of assigned package
            target_x = -1;
            for (int p = 0; p < MAX_TOTAL_PACKAGES; p++) {
                if (packageExists[p] && packages[p].assignedTruck == t && packages[p].status == 0) {
                    target_x = packages[p].pickup_x;
                    target_y = packages[p].pickup_y;
                    break;
                }
            }
            
            if (target_x == -1) {
                shmPtr->truckMovementInstructions[t] = 's';
                continue;
            }
        }
        
        char move = get_next_move(trucks[t].x, trucks[t].y, target_x, target_y);
        shmPtr->truckMovementInstructions[t] = move;
        
        // Get auth string if needed
        if (move != 's' && trucks[t].packageCount > 0) {
            find_auth_string(t, shmPtr->authStrings[t]);
        }
    }
}

int main() {
    init_system();
    
    TurnChangeResponse resp;
    TurnReadyRequest req;
    req.mtype = 1;
    
    // Receive initial state
    msgrcv(mainMsgId, &resp, sizeof(TurnChangeResponse) - sizeof(long), 2, 0);
    
    while (!resp.finished) {
        update_state(&resp);
        assign_packages();
        plan_moves();
        
        // Send ready signal
        msgsnd(mainMsgId, &req, sizeof(TurnReadyRequest) - sizeof(long), 0);
        
        // Wait for next turn
        msgrcv(mainMsgId, &resp, sizeof(TurnChangeResponse) - sizeof(long), 2, 0);
        
        if (resp.errorOccured) {
            fprintf(stderr, "Error occurred\n");
            break;
        }
    }
    
    // Cleanup
    shmdt(shmPtr);
    
    return 0;
}