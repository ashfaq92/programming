# Urban Air Mobility MAS Project Report

## 1. Agent Roles and Behaviors

### 1.1 CustomerAgent (JourneyRequester)
- **Role**: Submits journey requests and receives route proposals.
- **Behaviors**:
  - **OneShotBehaviour**: Sends `REQUEST_JOURNEY:X,Y,30min` to the Supervisor agent.
  - **CyclicBehaviour**: Listens for `PROPOSE_ROUTE` messages and logs received proposals.

### 1.2 SupervisorSoSAgent (JourneyCoordinator)
- **Role**: Coordinates journey planning and negotiates resource dispatch.
- **Behaviors**:
  - **CyclicBehaviour**: Waits for `REQUEST_JOURNEY` from Customer; sends `DISPATCH_REQUEST` to Scooter.
  - **CyclicBehaviour**: Processes Scooter's `ACCEPT` or `REJECT`, then sends `PROPOSE_ROUTE` back to Customer.

### 1.3 ScooterAgent (Vehicle Resource)
- **Role**: Decides whether to accept or reject dispatch requests based on internal state.
- **Behaviors**:
  - **CyclicBehaviour**: Waits for `DISPATCH_REQUEST`; if not busy, sets busy and replies `ACCEPT`, otherwise `REJECT`.

## 2. Communication Flow
1. **CustomerAgent** → SupervisorSoSAgent: `REQUEST_JOURNEY:X,Y,30min`
2. **SupervisorSoSAgent** → ScooterAgent: `DISPATCH_REQUEST`
3. **ScooterAgent** → SupervisorSoSAgent: `ACCEPT` or `REJECT`
4. **SupervisorSoSAgent** → CustomerAgent: `PROPOSE_ROUTE:scooter1 ETA=10min`
5. **CustomerAgent** logs the proposal.




