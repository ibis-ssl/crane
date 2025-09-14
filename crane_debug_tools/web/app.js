/**
 * Crane Robot Skills Debugger Web Application
 */

class CraneDebugger {
    constructor() {
        this.websocket = null;
        this.isConnected = false;
        this.availableSkills = [];
        this.selectedSkill = null;
        this.worldModel = null;
        this.robots = [];
        this.commandIndicatorTimeout = null;
        this.sessionActivated = false;
        this.executingSkills = new Map(); // robotId -> {skillName, startTime}
        this.selectedRobotId = null;
        this.lastRobotStatusUpdate = 0;
        this.robotStatusUpdateInterval = 500; // Update robot status max every 500ms
        this.isUpdatingRobotStatus = false;
        this.robotStatusUpdateTimer = null;
        this.lastFieldVisualizationUpdate = 0;
        this.fieldVisualizationUpdateInterval = 200; // Update field visualization max every 200ms
        this.isUpdatingFieldVisualization = false;
        this.fieldVisualizationUpdateTimer = null;
        this.isModalOpen = false;
        this.modalUpdateTimer = null;
        this.modalUpdateInterval = 100; // Update modal every 100ms for real-time feel

        this.initializeUI();
        this.connectWebSocket();
    }

    initializeUI() {
        // Populate robot ID selector
        const robotSelect = document.getElementById('robotId');
        for (let i = 0; i < 16; i++) {
            const option = document.createElement('option');
            option.value = i;
            option.textContent = `Robot ${i}`;
            robotSelect.appendChild(option);
        }

        // Event listeners
        document.getElementById('executeBtn').addEventListener('click', () => {
            this.executeSelectedSkill();
        });

        document.getElementById('clearLogBtn').addEventListener('click', () => {
            this.clearLog();
        });

        // Robot detail modal event listeners
        document.getElementById('executeSkillOnRobot').addEventListener('click', () => {
            this.executeSkillOnSelectedRobot();
        });

        // Set up event delegation for robot status cards (more reliable than individual listeners)
        document.getElementById('robotStatus').addEventListener('click', (e) => {
            const robotCard = e.target.closest('.robot-status-card');
            if (robotCard) {
                e.preventDefault();
                e.stopPropagation();
                const robotId = parseInt(robotCard.getAttribute('data-robot-id'));
                console.log('Robot status card clicked via delegation, robotId:', robotId);
                this.showRobotDetails(robotId);
            }
        });

        // Set up event delegation for field visualization robots
        document.getElementById('fieldViz').addEventListener('click', (e) => {
            const robotElement = e.target.closest('.robot');
            if (robotElement) {
                e.preventDefault();
                e.stopPropagation();
                const robotId = parseInt(robotElement.getAttribute('data-robot-id'));
                console.log('Field robot clicked via delegation, robotId:', robotId);
                this.showRobotDetails(robotId);
            }
        });

        // Set up modal event listeners for real-time updates
        const modalElement = document.getElementById('robotDetailModal');
        modalElement.addEventListener('shown.bs.modal', () => {
            console.log('Modal opened, starting real-time updates');
            this.isModalOpen = true;
            this.startModalRealTimeUpdates();
        });

        modalElement.addEventListener('hidden.bs.modal', () => {
            console.log('Modal closed, stopping real-time updates');
            this.isModalOpen = false;
            this.stopModalRealTimeUpdates();
        });

        this.addLog('System initialized', 'info');
    }

    connectWebSocket() {
        // Use current host for WebSocket connection instead of hardcoded localhost
        const wsUrl = `ws://${window.location.hostname}:8091`;
        this.addLog(`Connecting to ${wsUrl}...`, 'info');

        try {
            this.websocket = new WebSocket(wsUrl);

            this.websocket.onopen = () => {
                this.onWebSocketOpen();
            };

            this.websocket.onmessage = (event) => {
                this.onWebSocketMessage(event);
            };

            this.websocket.onclose = () => {
                this.onWebSocketClose();
            };

            this.websocket.onerror = (error) => {
                this.onWebSocketError(error);
            };

        } catch (error) {
            this.addLog(`WebSocket connection failed: ${error.message}`, 'error');
            this.updateConnectionStatus(false);

            // Fallback to demo mode
            this.addLog('Falling back to demo mode', 'warning');
            this.availableSkills = [
                "Sleep", "Idle", "Kick", "Receive", "Goalie", "Attacker", "SubAttacker",
                "StealBall", "SingleBallPlacement", "GoalKick", "SimpleKickOff",
                "KickOffSupport", "Marker", "TestMotionPosition",
                "TestMotionVelocity", "EmplaceRobot", "Forward", "BallNearbyPositioner",
                "GoOverBall", "SecondThreatDefender", "FreekickSaver", "PenaltyKick", "Teleop"
            ];
            this.populateSkillsList(this.availableSkills);
            this.addLog(`Demo mode: ${this.availableSkills.length} skills available`, 'info');
        }
    }

    onWebSocketOpen() {
        this.isConnected = true;
        this.updateConnectionStatus(true);
        this.addLog('Connected to crane debug server', 'success');

        // Immediately activate SimpleAI session on connection
        this.sendMessage({
            type: 'activate_simple_ai'
        });

        // Request available skills
        this.sendMessage({
            type: 'get_skills'
        });
    }

    onWebSocketMessage(event) {
        try {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        } catch (error) {
            this.addLog(`Failed to parse message: ${error.message}`, 'error');
        }
    }

    onWebSocketClose() {
        this.isConnected = false;
        this.updateConnectionStatus(false);
        this.addLog('Disconnected from server', 'warning');

        // Attempt to reconnect after 3 seconds
        setTimeout(() => {
            if (!this.isConnected) {
                this.addLog('Attempting to reconnect...', 'info');
                this.connectWebSocket();
            }
        }, 3000);
    }

    onWebSocketError(error) {
        this.addLog(`WebSocket error: ${error.message || 'Connection failed'}`, 'error');
        this.updateConnectionStatus(false);
    }

    handleMessage(message) {
        switch (message.type) {
            case 'available_skills':
                this.handleAvailableSkills(message.skills);
                break;
            case 'world_model':
                this.handleWorldModel(message);
                break;
            case 'robot_commands':
                this.handleRobotCommands(message.commands);
                break;
            case 'skill_execution_started':
                this.addLog(`Skill execution started: ${message.skill_name} on robot ${message.robot_id}`, 'info');
                this.executingSkills.set(message.robot_id, {
                    skillName: message.skill_name,
                    startTime: new Date()
                });
                this.scheduleRobotStatusUpdate();
                break;
            case 'skill_goal_response':
                if (message.accepted) {
                    this.addLog('Skill goal accepted by server', 'success');
                } else {
                    this.addLog('Skill goal rejected by server', 'error');
                }
                break;
            case 'skill_feedback':
                this.addLog(`Feedback: ${message.message}`, 'info');
                break;
            case 'skill_result':
                this.handleSkillResult(message);
                break;
            case 'error':
                this.addLog(`Error: ${message.message}`, 'error');
                break;
            case 'simple_ai_activated':
                this.sessionActivated = true;
                this.addLog('SimpleAI session activated', 'success');
                break;
            default:
                this.addLog(`Unknown message type: ${message.type}`, 'warning');
        }
    }

    handleAvailableSkills(skills) {
        this.availableSkills = skills;
        this.populateSkillsList(skills);
        this.addLog(`Loaded ${skills.length} available skills`, 'success');
    }

    handleWorldModel(worldModel) {
        this.worldModel = worldModel;

        // Throttled updates
        this.scheduleFieldVisualizationUpdate();
        this.scheduleRobotStatusUpdate();
    }

    handleRobotCommands(commands) {
        // Update robot command visualization and show indicator
        this.showCommandIndicator();

        // Update executing skills from robot commands state_factors
        if (commands && Array.isArray(commands)) {
            commands.forEach(cmd => {
                const robotId = cmd.robot_id;

                // Look for skill information in state_factors
                if (cmd.state_factors && cmd.state_factors.length > 0) {
                    // Find the main skill (usually the first factor or one with specific pattern)
                    const skillFactor = cmd.state_factors.find(factor =>
                        factor.name && factor.value === 'RUNNING'
                    ) || cmd.state_factors[0]; // fallback to first factor

                    if (skillFactor && skillFactor.name) {
                        // Update executing skills map
                        if (!this.executingSkills.has(robotId) ||
                            this.executingSkills.get(robotId).skillName !== skillFactor.name) {
                            this.executingSkills.set(robotId, {
                                skillName: skillFactor.name,
                                startTime: new Date(),
                                plannerName: cmd.planner_name || 'Unknown'
                            });
                        }
                    }
                } else {
                    // No state factors or empty - robot is idle
                    this.executingSkills.delete(robotId);
                }
            });

            // Update robot status display to reflect current skills (throttled)
            this.scheduleRobotStatusUpdate();
        }
    }

    handleSkillResult(result) {
        const codeNames = {
            1: 'SUCCEEDED',
            2: 'ABORTED',
            3: 'CANCELED'
        };

        const codeName = codeNames[result.code] || 'UNKNOWN';
        const logType = result.code === 1 ? 'success' : 'error';

        this.addLog(`Skill execution ${codeName} (result: ${result.result})`, logType);

        // Clear executing skill when it finishes
        if (result.robot_id !== undefined) {
            this.executingSkills.delete(result.robot_id);
        }

        // Update robot status display to reflect skill completion (throttled)
        this.scheduleRobotStatusUpdate();
    }

    sendMessage(message) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(message));
        } else {
            this.addLog('Cannot send message: not connected', 'error');
        }
    }

    populateSkillsList(skills) {
        const skillsList = document.getElementById('skillsList');
        skillsList.innerHTML = '';

        skills.forEach(skill => {
            const skillCard = document.createElement('button');
            skillCard.className = 'btn btn-outline-primary skill-card';
            skillCard.textContent = skill;
            skillCard.addEventListener('click', (event) => {
                this.selectSkill(skill, event.target);
            });
            skillsList.appendChild(skillCard);
        });
    }

    selectSkill(skillName, targetElement = null) {
        this.selectedSkill = skillName;
        document.getElementById('selectedSkill').value = skillName;
        document.getElementById('executeBtn').disabled = false;

        // Remove active class from all skill cards
        document.querySelectorAll('.skill-card').forEach(card => {
            card.classList.remove('btn-primary');
            card.classList.add('btn-outline-primary');
        });

        // Add active class to selected skill if target element is provided
        if (targetElement) {
            targetElement.classList.remove('btn-outline-primary');
            targetElement.classList.add('btn-primary');
        }

        this.generateParametersForm(skillName);
        this.addLog(`Selected skill: ${skillName}`, 'info');
    }

    generateParametersForm(skillName) {
        const parametersForm = document.getElementById('parametersForm');

        // Define common parameters for different skills
        const skillParameters = {
            'Kick': [
                { name: 'target', type: 'text', label: 'Target (x,y)', value: '0,0' },
                { name: 'kick_power', type: 'number', label: 'Kick Power', min: '0', max: '1', step: '0.05', value: '0.7' },
                { name: 'use_target_kick_speed', type: 'checkbox', label: 'Use Target Kick Speed', value: 'false' },
                { name: 'target_kick_speed', type: 'number', label: 'Target Kick Speed', min: '0', max: '10', step: '0.1', value: '2.0' },
                { name: 'use_target_chip_distance', type: 'checkbox', label: 'Use Target Chip Distance', value: 'false' },
                { name: 'target_chip_distance', type: 'number', label: 'Target Chip Distance', min: '0', max: '10', step: '0.1', value: '2.0' },
                { name: 'chip_kick', type: 'checkbox', label: 'Chip Kick', value: 'false' },
                { name: 'with_dribble', type: 'checkbox', label: 'With Dribble', value: 'false' },
                { name: 'dribble_power', type: 'number', label: 'Dribble Power', min: '0', max: '1', step: '0.05', value: '0.3' },
                { name: 'angle_threshold', type: 'number', label: 'Angle Threshold', min: '0', max: '1', step: '0.01', value: '0.1' },
                { name: 'around_interval', type: 'number', label: 'Around Interval', min: '0', max: '1', step: '0.01', value: '0.15' },
                { name: 'go_around_ball', type: 'checkbox', label: 'Go Around Ball', value: 'true' },
                { name: 'moving_speed_threshold', type: 'number', label: 'Moving Speed Threshold', min: '0', max: '5', step: '0.1', value: '0.2' },
                { name: 'kicked_speed_threshold', type: 'number', label: 'Kicked Speed Threshold', min: '0', max: '10', step: '0.1', value: '1.5' }
            ],
            'EmplaceRobot': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' },
                { name: 'target_theta', type: 'number', label: 'Target Theta', step: '0.1' }
            ],
            'Sleep': [
                { name: 'duration', type: 'number', label: 'Duration (seconds)', min: '0', step: '0.1', value: '1.0' }
            ],
            'StealBall': [
                { name: 'steal_method', type: 'text', label: 'Steal Method', value: 'side' },
                { name: 'kicker_power', type: 'number', label: 'Kicker Power', min: '0', max: '1', step: '0.1', value: '0.4' }
            ],
            'SubAttacker': [
                { name: 'ball_vel_threshold', type: 'number', label: 'Ball Velocity Threshold', min: '0', max: '5', step: '0.1', value: '0.2' },
                { name: 'kicker_power', type: 'number', label: 'Kicker Power', min: '0', max: '1', step: '0.1', value: '0.8' }
            ],
            'SecondThreatDefender': [
                { name: 'offset', type: 'number', label: 'Offset', min: '0', max: '2', step: '0.1', value: '0.3' }
            ],
            'KickoffAttack': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1', value: '0.0' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1', value: '1.0' },
                { name: 'kick_power', type: 'number', label: 'Kick Power', min: '0', max: '1', step: '0.05', value: '0.25' }
            ],
            'BallNearbyPositioner': [
                { name: 'margin_distance', type: 'number', label: 'Margin Distance', min: '0', max: '5', step: '0.1' },
                { name: 'total_robot_number', type: 'number', label: 'Total Robot Number', min: '1', max: '15', step: '1' },
                { name: 'current_robot_index', type: 'number', label: 'Current Robot Index', min: '0', max: '14', step: '1' },
                { name: 'robot_interval', type: 'number', label: 'Robot Interval', min: '0.1', max: '2', step: '0.1' },
                { name: 'alternative_target_mode', type: 'checkbox', label: 'Alternative Target Mode', value: 'false' },
                { name: 'positioning_policy', type: 'text', label: 'Positioning Policy', value: 'auto' },
                { name: 'line_policy', type: 'text', label: 'Line Policy', value: 'arc' }
            ],
            'Forward': [
                { name: 'front_point', type: 'text', label: 'Front Point (x,y)', value: '0,0' },
                { name: 'back_point', type: 'text', label: 'Back Point (x,y)', value: '0,0' },
                { name: 'max_ball_distance', type: 'number', label: 'Max Ball Distance', min: '0', max: '10', step: '0.1' },
                { name: 'max_vel', type: 'number', label: 'Max Velocity', min: '0', max: '10', step: '0.1' }
            ],
            'GoOverBall': [
                { name: 'next_target_x', type: 'number', label: 'Next Target X', step: '0.1' },
                { name: 'next_target_y', type: 'number', label: 'Next Target Y', step: '0.1' },
                { name: 'margin', type: 'number', label: 'Margin', min: '0', max: '1', step: '0.05' },
                { name: 'reach_threshold', type: 'number', label: 'Reach Threshold', min: '0', max: '1', step: '0.05' }
            ],
            'Teleop': [
                { name: 'rotation_deg', type: 'number', label: 'Rotation (degrees)', min: '-180', max: '180', step: '1', value: '0' },
                { name: 'use_local_coordinate', type: 'checkbox', label: 'Use Local Coordinate', value: 'false' }
            ],
            'TestMotionPosition': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' }
            ],
            'TestMotionVelocity': [
                { name: 'velocity_x', type: 'number', label: 'Velocity X', step: '0.1' },
                { name: 'velocity_y', type: 'number', label: 'Velocity Y', step: '0.1' }
            ],
            'Receive': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' }
            ],
            'Attacker': [
                { name: 'moving_ball_velocity', type: 'number', label: 'Moving Ball Velocity', min: '0', max: '5', step: '0.1', value: '1.0' },
                { name: 'robot_acc_for_prediction', type: 'number', label: 'Robot Acc for Prediction', min: '0', max: '10', step: '0.1', value: '2.5' },
                { name: 'robot_max_vel_for_prediction', type: 'number', label: 'Robot Max Vel for Prediction', min: '0', max: '10', step: '0.1', value: '5.0' }
            ],
            'Goalie': [
                { name: 'run_inplay', type: 'checkbox', label: 'Run in play', value: 'true' },
                { name: 'block_distance', type: 'number', label: 'Block Distance', min: '0.1', max: '2.0', step: '0.1', value: '0.5' },
                { name: 'robot_acc_for_prediction', type: 'number', label: 'Robot Acc for Prediction', min: '0.1', max: '10.0', step: '0.1', value: '2.0' },
                { name: 'robot_max_vel_for_prediction', type: 'number', label: 'Robot Max Vel for Prediction', min: '0.1', max: '10.0', step: '0.1', value: '5.0' }
            ],
            'SingleBallPlacement': [
                { name: 'placement_x', type: 'number', label: 'Placement X', step: '0.1', value: '0.0' },
                { name: 'placement_y', type: 'number', label: 'Placement Y', step: '0.1', value: '0.0' },
                { name: 'pass_enable', type: 'checkbox', label: 'Pass Enable', value: 'false' },
                { name: 'コート端判定のオフセット', type: 'number', label: 'Court Edge Offset', min: '0', max: '1', step: '0.05', value: '0.0' }
            ],
            'Marker': [
                { name: 'marking_robot_id', type: 'number', label: 'Marking Robot ID', min: '0', max: '15', step: '1', value: '0' },
                { name: 'mark_distance', type: 'number', label: 'Mark Distance', min: '0.1', max: '3.0', step: '0.1', value: '0.5' },
                { name: 'mark_mode', type: 'text', label: 'Mark Mode', value: 'save_goal' }
            ],
            'PenaltyKick': [
                { name: 'start_from_kick', type: 'checkbox', label: 'Start From Kick', value: 'false' },
                { name: 'prepare_margin', type: 'number', label: 'Prepare Margin', min: '0.1', max: '2.0', step: '0.1', value: '0.6' }
            ],
            'GoalKick': [
                { name: 'キック角度の最低要求精度[deg]', type: 'number', label: 'Kick Angle Accuracy (deg)', min: '0.1', max: '10', step: '0.1', value: '1.0' },
                { name: 'dribble_power', type: 'number', label: 'Dribble Power', min: '0', max: '1', step: '0.1', value: '0.0' }
            ]
        };

        const parameters = skillParameters[skillName] || [];

        if (parameters.length === 0) {
            parametersForm.innerHTML = '<p class="text-muted">No configurable parameters for this skill</p>';
            return;
        }

        let formHTML = '';
        parameters.forEach(param => {
            if (param.type === 'checkbox') {
                formHTML += `
                    <div class="mb-3 form-check">
                        <input type="checkbox"
                               class="form-check-input"
                               id="param_${param.name}"
                               name="${param.name}"
                               ${param.value === 'true' ? 'checked' : ''}>
                        <label class="form-check-label" for="param_${param.name}">
                            ${param.label}
                        </label>
                    </div>
                `;
            } else {
                formHTML += `
                    <div class="mb-3">
                        <label for="param_${param.name}" class="form-label">${param.label}</label>
                        <input type="${param.type}"
                               class="form-control"
                               id="param_${param.name}"
                               name="${param.name}"
                               ${param.min ? `min="${param.min}"` : ''}
                               ${param.max ? `max="${param.max}"` : ''}
                               ${param.step ? `step="${param.step}"` : ''}
                               ${param.value ? `value="${param.value}"` : ''}>
                    </div>
                `;
            }
        });

        parametersForm.innerHTML = formHTML;
    }

    executeSelectedSkill() {
        if (!this.selectedSkill) {
            this.addLog('No skill selected', 'warning');
            return;
        }

        const robotId = parseInt(document.getElementById('robotId').value);
        const parameters = this.collectParameters();

        // Only activate session once, not every time
        if (!this.sessionActivated) {
            this.sendMessage({
                type: 'activate_simple_ai'
            });
            this.sessionActivated = true;

            // Only wait on first activation
            setTimeout(() => {
                this.executeSkillInternal(robotId, parameters);
            }, 50);  // Reduced from 100ms to 50ms
        } else {
            // Direct execution without delay
            this.executeSkillInternal(robotId, parameters);
        }
    }

    executeSkillInternal(robotId, parameters) {
        const message = {
            type: 'execute_skill',
            skill_name: this.selectedSkill,
            robot_id: robotId,
            parameters: parameters
        };

        this.sendMessage(message);
        this.addLog(`Executing ${this.selectedSkill} on robot ${robotId}...`, 'info');
    }

    collectParameters() {
        const parametersForm = document.getElementById('parametersForm');
        const inputs = parametersForm.querySelectorAll('input');
        const parameters = [];

        inputs.forEach(input => {
            if (input.type === 'checkbox') {
                // Always include checkbox parameters
                parameters.push({
                    name: input.name,
                    value: input.checked ? 'true' : 'false'
                });
            } else if (input.value.trim() !== '') {
                parameters.push({
                    name: input.name,
                    value: input.value
                });
            }
        });

        return parameters;
    }

    updateFieldVisualization() {
        if (!this.worldModel || this.isUpdatingFieldVisualization) return;

        // Throttle updates to prevent excessive DOM manipulation
        const now = Date.now();
        if (now - this.lastFieldVisualizationUpdate < this.fieldVisualizationUpdateInterval) {
            return;
        }

        this.isUpdatingFieldVisualization = true;
        this.lastFieldVisualizationUpdate = now;

        const field = document.getElementById('fieldViz');
        const fieldRect = field.getBoundingClientRect();

        // Clear previous robots and ball
        field.querySelectorAll('.robot, .ball').forEach(el => el.remove());

        // Initialize field lines if not already present
        this.initializeFieldLines(field, fieldRect.width, fieldRect.height);

        // Field dimensions (SSL field is 12m x 9m)
        const fieldWidth = 12; // meters
        const fieldHeight = 9; // meters

        // Scale world coordinates to screen coordinates
        const scaleX = fieldRect.width / fieldWidth;
        const scaleY = fieldRect.height / fieldHeight;

        // Determine team colors based on is_yellow flag
        const isYellow = this.worldModel.is_yellow || false;
        const ourColor = isYellow ? '#ffc107' : '#007bff';  // Yellow or Blue
        const theirColor = isYellow ? '#007bff' : '#ffc107';  // Blue or Yellow

        // Draw ball
        if (this.worldModel.ball) {
            const ball = document.createElement('div');
            ball.className = 'ball';
            ball.style.left = `${(this.worldModel.ball.x + fieldWidth/2) * scaleX}px`;
            ball.style.top = `${(fieldHeight/2 - this.worldModel.ball.y) * scaleY}px`;
            field.appendChild(ball);
        }

        // Draw our robots
        if (this.worldModel.robots_ours) {
            this.worldModel.robots_ours.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-ours';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = ourColor;
                robotEl.style.cursor = 'pointer';
                robotEl.title = `Our Robot ${robot.id} - Click for details`;
                robotEl.setAttribute('data-robot-id', robot.id);

                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';

                field.appendChild(robotEl);
            });
        }

        // Draw their robots
        if (this.worldModel.robots_theirs) {
            this.worldModel.robots_theirs.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-theirs';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = theirColor;
                robotEl.style.cursor = 'pointer';
                robotEl.title = `Their Robot ${robot.id} - Click for details`;
                robotEl.setAttribute('data-robot-id', robot.id);

                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';

                field.appendChild(robotEl);
            });
        }

        // Fallback for old format (backward compatibility)
        if (this.worldModel.robots && !this.worldModel.robots_ours) {
            this.worldModel.robots.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-ours';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = ourColor;
                robotEl.style.cursor = 'pointer';
                robotEl.title = `Robot ${robot.id} - Click for details`;
                robotEl.setAttribute('data-robot-id', robot.id);

                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';

                field.appendChild(robotEl);
            });
        }

        // Field visualization update complete
        this.isUpdatingFieldVisualization = false;
    }

    initializeFieldLines(field, pixelWidth, pixelHeight) {
        // Check if field lines already exist
        if (field.querySelector('.field-line')) {
            return;
        }

        // SSL field dimensions: 12m x 9m
        const fieldWidth = 12; // meters
        const fieldHeight = 9; // meters

        // Scale world coordinates to screen coordinates
        const scaleX = pixelWidth / fieldWidth;
        const scaleY = pixelHeight / fieldHeight;

        // Center line (vertical)
        const centerLine = document.createElement('div');
        centerLine.className = 'field-line field-line-vertical';
        centerLine.style.left = `${pixelWidth / 2 - 1}px`;
        centerLine.style.top = '0px';
        field.appendChild(centerLine);

        // Center circle (1m radius in SSL)
        const centerCircle = document.createElement('div');
        centerCircle.className = 'field-center-circle';
        const circleRadius = 1.0 * Math.min(scaleX, scaleY); // 1m radius
        centerCircle.style.width = `${circleRadius * 2}px`;
        centerCircle.style.height = `${circleRadius * 2}px`;
        centerCircle.style.left = `${pixelWidth / 2 - circleRadius}px`;
        centerCircle.style.top = `${pixelHeight / 2 - circleRadius}px`;
        field.appendChild(centerCircle);

        // Goal areas (0.7m wide x 0.35m deep in SSL)
        const goalWidth = 0.7 * scaleY; // 0.7m goal width
        const goalDepth = 0.35 * scaleX; // 0.35m goal depth

        // Left goal area
        const leftGoalArea = document.createElement('div');
        leftGoalArea.className = 'field-goal-area';
        leftGoalArea.style.width = `${goalDepth}px`;
        leftGoalArea.style.height = `${goalWidth}px`;
        leftGoalArea.style.left = '0px';
        leftGoalArea.style.top = `${(pixelHeight - goalWidth) / 2}px`;
        field.appendChild(leftGoalArea);

        // Right goal area
        const rightGoalArea = document.createElement('div');
        rightGoalArea.className = 'field-goal-area';
        rightGoalArea.style.width = `${goalDepth}px`;
        rightGoalArea.style.height = `${goalWidth}px`;
        rightGoalArea.style.right = '0px';
        rightGoalArea.style.top = `${(pixelHeight - goalWidth) / 2}px`;
        field.appendChild(rightGoalArea);

        // Penalty areas (2m wide x 1m deep in SSL)
        const penaltyWidth = 2.0 * scaleY; // 2m penalty width
        const penaltyDepth = 1.0 * scaleX; // 1m penalty depth

        // Left penalty area
        const leftPenaltyArea = document.createElement('div');
        leftPenaltyArea.className = 'field-penalty-area';
        leftPenaltyArea.style.width = `${penaltyDepth}px`;
        leftPenaltyArea.style.height = `${penaltyWidth}px`;
        leftPenaltyArea.style.left = '0px';
        leftPenaltyArea.style.top = `${(pixelHeight - penaltyWidth) / 2}px`;
        field.appendChild(leftPenaltyArea);

        // Right penalty area
        const rightPenaltyArea = document.createElement('div');
        rightPenaltyArea.className = 'field-penalty-area';
        rightPenaltyArea.style.width = `${penaltyDepth}px`;
        rightPenaltyArea.style.height = `${penaltyWidth}px`;
        rightPenaltyArea.style.right = '0px';
        rightPenaltyArea.style.top = `${(pixelHeight - penaltyWidth) / 2}px`;
        field.appendChild(rightPenaltyArea);
    }

    updateRobotStatus() {
        if (!this.worldModel || this.isUpdatingRobotStatus) {
            return;
        }

        // Throttle updates to prevent excessive DOM manipulation
        const now = Date.now();
        if (now - this.lastRobotStatusUpdate < this.robotStatusUpdateInterval) {
            return;
        }

        this.isUpdatingRobotStatus = true;
        this.lastRobotStatusUpdate = now;

        const statusContainer = document.getElementById('robotStatus');
        let statusHTML = '';

        // Helper function to create compact robot grid
        const createRobotGrid = (robots, title, color) => {
            if (!robots || robots.length === 0) return '';

            let html = `<h6 class="${title.includes('Our') ? 'text-primary' : 'text-danger'} mb-2">${title}</h6>`;
            html += '<div class="row g-1 mb-3">';

            robots.forEach(robot => {
                const executingSkill = this.executingSkills.get(robot.id);
                let skillInfo;

                if (executingSkill) {
                    const plannerInfo = executingSkill.plannerName && executingSkill.plannerName !== 'Unknown'
                        ? `<div class="text-info" style="font-size: 8px; margin-top: 1px;">${executingSkill.plannerName}</div>`
                        : '';
                    skillInfo = `
                        <div class="text-success" style="font-size: 9px; font-weight: bold; margin-top: 2px;">▶ ${executingSkill.skillName}</div>
                        ${plannerInfo}
                    `;
                } else {
                    skillInfo = '<div class="text-muted" style="font-size: 9px; margin-top: 2px;">Idle</div>';
                }

                html += `
                    <div class="col-4">
                        <div class="p-2 border rounded-2 robot-status-card text-center"
                             style="border-left: 3px solid ${color} !important; cursor: pointer; min-height: 50px; display: flex; flex-direction: column; justify-content: center;"
                             data-robot-id="${robot.id}"
                             title="Robot ${robot.id} - Click for details">
                            <div class="fw-bold" style="font-size: 14px;">R${robot.id}</div>
                            ${skillInfo}
                        </div>
                    </div>
                `;
            });

            html += '</div>';
            return html;
        };

        // Display our robots
        if (this.worldModel.robots_ours && this.worldModel.robots_ours.length > 0) {
            const isYellow = this.worldModel.is_yellow || false;
            const ourColorName = isYellow ? 'Yellow' : 'Blue';
            const ourColor = isYellow ? '#ffc107' : '#007bff';

            statusHTML += createRobotGrid(
                this.worldModel.robots_ours,
                `Our Robots (${ourColorName})`,
                ourColor
            );
        }

        // Display their robots
        if (this.worldModel.robots_theirs && this.worldModel.robots_theirs.length > 0) {
            const isYellow = this.worldModel.is_yellow || false;
            const theirColorName = isYellow ? 'Blue' : 'Yellow';
            const theirColor = isYellow ? '#007bff' : '#ffc107';

            statusHTML += createRobotGrid(
                this.worldModel.robots_theirs,
                `Their Robots (${theirColorName})`,
                theirColor
            );
        }

        // Fallback for old format
        if (statusHTML === '' && this.worldModel.robots && this.worldModel.robots.length > 0) {
            statusHTML += createRobotGrid(this.worldModel.robots, 'Robots', '#007bff');
        }

        if (statusHTML === '') {
            statusHTML = '<p class="text-muted">No robot data available</p>';
        }

        statusContainer.innerHTML = statusHTML;

        // Event listeners are handled by delegation in initializeUI()
        // No need to add individual listeners here since DOM gets recreated frequently

        this.isUpdatingRobotStatus = false;
    }

    updateConnectionStatus(connected) {
        const statusIndicator = document.getElementById('connectionStatus');
        const statusText = document.getElementById('connectionText');

        if (connected) {
            statusIndicator.className = 'status-indicator connected';
            statusText.textContent = 'Connected';
        } else {
            statusIndicator.className = 'status-indicator disconnected';
            statusText.textContent = 'Disconnected';
        }
    }

    addLog(message, type = 'info') {
        const logContainer = document.getElementById('logContainer');
        const timestamp = new Date().toLocaleTimeString();

        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;
        logEntry.innerHTML = `<span class="text-muted">[${timestamp}]</span> ${message}`;

        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;

        // Limit log entries to 500 for better performance
        const entries = logContainer.children;
        if (entries.length > 500) {
            logContainer.removeChild(entries[0]);
        }
    }

    clearLog() {
        document.getElementById('logContainer').innerHTML = '';
        this.addLog('Log cleared', 'info');
    }

    showCommandIndicator() {
        const indicator = document.getElementById('commandIndicator');
        const text = document.getElementById('commandText');

        // Show indicator as active
        indicator.className = 'command-indicator active';
        text.textContent = 'Command Received';

        // Clear previous timeout
        if (this.commandIndicatorTimeout) {
            clearTimeout(this.commandIndicatorTimeout);
        }

        // Auto-hide after 200ms
        this.commandIndicatorTimeout = setTimeout(() => {
            indicator.className = 'command-indicator';
            text.textContent = 'Waiting...';
        }, 200);
    }

    scheduleRobotStatusUpdate() {
        // Cancel any pending update
        if (this.robotStatusUpdateTimer) {
            clearTimeout(this.robotStatusUpdateTimer);
        }

        const now = Date.now();
        const timeSinceLastUpdate = now - this.lastRobotStatusUpdate;

        if (timeSinceLastUpdate >= this.robotStatusUpdateInterval) {
            // Update immediately if enough time has passed
            this.updateRobotStatus();
        } else {
            // Schedule update for later
            const delay = this.robotStatusUpdateInterval - timeSinceLastUpdate;
            this.robotStatusUpdateTimer = setTimeout(() => {
                this.updateRobotStatus();
            }, delay);
        }
    }

    scheduleFieldVisualizationUpdate() {
        // Cancel any pending update
        if (this.fieldVisualizationUpdateTimer) {
            clearTimeout(this.fieldVisualizationUpdateTimer);
        }

        const now = Date.now();
        const timeSinceLastUpdate = now - this.lastFieldVisualizationUpdate;

        if (timeSinceLastUpdate >= this.fieldVisualizationUpdateInterval) {
            // Update immediately if enough time has passed
            this.updateFieldVisualization();
        } else {
            // Schedule update for later
            const delay = this.fieldVisualizationUpdateInterval - timeSinceLastUpdate;
            this.fieldVisualizationUpdateTimer = setTimeout(() => {
                this.updateFieldVisualization();
            }, delay);
        }
    }

    startModalRealTimeUpdates() {
        if (this.modalUpdateTimer) {
            clearInterval(this.modalUpdateTimer);
        }

        this.modalUpdateTimer = setInterval(() => {
            if (this.isModalOpen && this.selectedRobotId !== null) {
                this.updateModalContent();
            }
        }, this.modalUpdateInterval);
    }

    stopModalRealTimeUpdates() {
        if (this.modalUpdateTimer) {
            clearInterval(this.modalUpdateTimer);
            this.modalUpdateTimer = null;
        }
    }

    updateModalContent() {
        if (!this.isModalOpen || this.selectedRobotId === null || !this.worldModel) {
            return;
        }

        // Find current robot data
        let robotData = null;
        let teamType = 'unknown';

        if (this.worldModel.robots_ours) {
            robotData = this.worldModel.robots_ours.find(r => r.id === this.selectedRobotId);
            if (robotData) teamType = 'ours';
        }
        if (!robotData && this.worldModel.robots_theirs) {
            robotData = this.worldModel.robots_theirs.find(r => r.id === this.selectedRobotId);
            if (robotData) teamType = 'theirs';
        }
        if (!robotData && this.worldModel.robots) {
            robotData = this.worldModel.robots.find(r => r.id === this.selectedRobotId);
            if (robotData) teamType = 'ours';
        }

        if (!robotData) {
            console.warn(`Robot ${this.selectedRobotId} not found in current world model`);
            return;
        }

        // Update position, velocity, angle data using specific IDs
        const positionData = document.getElementById('positionData');
        if (positionData) {
            positionData.textContent = `(${robotData.x.toFixed(3)}, ${robotData.y.toFixed(3)})`;
        }

        const velocityData = document.getElementById('velocityData');
        if (velocityData) {
            velocityData.textContent = `(${robotData.vx.toFixed(3)}, ${robotData.vy.toFixed(3)})`;
        }

        const angleData = document.getElementById('angleData');
        if (angleData) {
            angleData.textContent = `${robotData.theta.toFixed(3)} rad (${(robotData.theta * 180 / Math.PI).toFixed(1)}°)`;
        }

        const speedData = document.getElementById('speedData');
        if (speedData) {
            const speed = Math.sqrt(robotData.vx * robotData.vx + robotData.vy * robotData.vy);
            speedData.textContent = `${speed.toFixed(3)} m/s`;
        }

        // Update angular velocity if exists
        const angularVelData = document.getElementById('angularVelData');
        if (angularVelData && robotData.omega !== undefined) {
            angularVelData.textContent = `${robotData.omega.toFixed(3)} rad/s`;
        }

        // Update distance from ball
        const distanceSpan = document.getElementById('distanceFromBall');
        if (distanceSpan && this.worldModel.ball) {
            const ballX = this.worldModel.ball.x;
            const ballY = this.worldModel.ball.y;
            const distance = Math.sqrt(
                Math.pow(robotData.x - ballX, 2) +
                Math.pow(robotData.y - ballY, 2)
            );
            distanceSpan.textContent = `${distance.toFixed(3)} m`;
        }

        // Update last updated time
        const lastUpdatedSpan = document.getElementById('lastUpdated');
        if (lastUpdatedSpan) {
            lastUpdatedSpan.textContent = new Date().toLocaleTimeString();
        }

        // Update skill information
        const executingSkill = this.executingSkills.get(this.selectedRobotId);
        const skillAlert = document.getElementById('skillStatusAlert');
        if (skillAlert) {
            if (executingSkill) {
                skillAlert.className = 'alert alert-success';
                skillAlert.innerHTML = `
                    <h6><i class="fas fa-play-circle"></i> Currently Executing</h6>
                    <strong>Skill:</strong> ${executingSkill.skillName}<br>
                    ${executingSkill.plannerName ? `<strong>Planner:</strong> ${executingSkill.plannerName}<br>` : ''}
                    <strong>Started:</strong> ${executingSkill.startTime.toLocaleTimeString()}
                `;
            } else {
                skillAlert.className = 'alert alert-secondary';
                skillAlert.innerHTML = `
                    <h6><i class="fas fa-pause-circle"></i> Status</h6>
                    Robot is currently idle
                `;
            }
        }
    }

    showRobotDetails(robotId) {
        console.log('showRobotDetails called with robotId:', robotId);
        this.selectedRobotId = robotId;

        // Find robot data
        let robotData = null;
        let teamType = 'unknown';

        if (this.worldModel) {
            if (this.worldModel.robots_ours) {
                robotData = this.worldModel.robots_ours.find(r => r.id === robotId);
                if (robotData) teamType = 'ours';
            }
            if (!robotData && this.worldModel.robots_theirs) {
                robotData = this.worldModel.robots_theirs.find(r => r.id === robotId);
                if (robotData) teamType = 'theirs';
            }
            if (!robotData && this.worldModel.robots) {
                robotData = this.worldModel.robots.find(r => r.id === robotId);
                if (robotData) teamType = 'ours';
            }
        }

        if (!robotData) {
            this.addLog(`Robot ${robotId} not found in world model`, 'warning');
            return;
        }

        // Get executing skill info
        const executingSkill = this.executingSkills.get(robotId);

        // Build detail content
        const isYellow = this.worldModel?.is_yellow || false;
        const teamColor = teamType === 'ours' ?
            (isYellow ? '#ffc107' : '#007bff') :
            (isYellow ? '#007bff' : '#ffc107');
        const teamName = teamType === 'ours' ?
            `Our Robot (${isYellow ? 'Yellow' : 'Blue'})` :
            `Their Robot (${isYellow ? 'Blue' : 'Yellow'})`;

        const skillInfo = executingSkill ? `
            <div class="alert alert-success" id="skillStatusAlert">
                <h6><i class="fas fa-play-circle"></i> Currently Executing</h6>
                <strong>Skill:</strong> ${executingSkill.skillName}<br>
                ${executingSkill.plannerName ? `<strong>Planner:</strong> ${executingSkill.plannerName}<br>` : ''}
                <strong>Started:</strong> ${executingSkill.startTime.toLocaleTimeString()}
            </div>
        ` : `
            <div class="alert alert-secondary" id="skillStatusAlert">
                <h6><i class="fas fa-pause-circle"></i> Status</h6>
                Robot is currently idle
            </div>
        `;

        const detailContent = `
            <div class="row">
                <div class="col-md-6">
                    <div class="card">
                        <div class="card-header" style="background-color: ${teamColor}; color: white;">
                            <h6 class="mb-0"><i class="fas fa-robot"></i> Robot ${robotId}</h6>
                            <small>${teamName}</small>
                        </div>
                        <div class="card-body">
                            <table class="table table-sm" id="robotDataTable">
                                <tr>
                                    <td><strong>Position:</strong></td>
                                    <td id="positionData">(${robotData.x.toFixed(3)}, ${robotData.y.toFixed(3)})</td>
                                </tr>
                                <tr>
                                    <td><strong>Velocity:</strong></td>
                                    <td id="velocityData">(${robotData.vx.toFixed(3)}, ${robotData.vy.toFixed(3)})</td>
                                </tr>
                                <tr>
                                    <td><strong>Angle:</strong></td>
                                    <td id="angleData">${robotData.theta.toFixed(3)} rad (${(robotData.theta * 180 / Math.PI).toFixed(1)}°)</td>
                                </tr>
                                <tr>
                                    <td><strong>Speed:</strong></td>
                                    <td id="speedData">${Math.sqrt(robotData.vx * robotData.vx + robotData.vy * robotData.vy).toFixed(3)} m/s</td>
                                </tr>
                                ${robotData.omega !== undefined ? `
                                <tr>
                                    <td><strong>Angular Vel:</strong></td>
                                    <td id="angularVelData">${robotData.omega.toFixed(3)} rad/s</td>
                                </tr>
                                ` : ''}
                            </table>
                        </div>
                    </div>
                </div>
                <div class="col-md-6">
                    ${skillInfo}

                    <div class="card">
                        <div class="card-header">
                            <h6 class="mb-0"><i class="fas fa-chart-line"></i> Real-time Data</h6>
                        </div>
                        <div class="card-body">
                            <div class="mb-2">
                                <strong>Distance from Ball:</strong>
                                <span id="distanceFromBall">Calculating...</span>
                            </div>
                            <div class="mb-2">
                                <strong>Last Updated:</strong>
                                <span id="lastUpdated">${new Date().toLocaleTimeString()}</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;

        // Update modal content
        document.getElementById('robotDetailModalLabel').textContent = `Robot ${robotId} Details`;
        document.getElementById('robotDetailContent').innerHTML = detailContent;

        // Calculate distance from ball
        if (this.worldModel?.ball) {
            const ballX = this.worldModel.ball.x;
            const ballY = this.worldModel.ball.y;
            const distance = Math.sqrt(
                Math.pow(robotData.x - ballX, 2) +
                Math.pow(robotData.y - ballY, 2)
            );
            document.getElementById('distanceFromBall').textContent = `${distance.toFixed(3)} m`;
        }

        // Enable execute skill button only for our robots
        const executeBtn = document.getElementById('executeSkillOnRobot');
        if (teamType === 'ours' && this.selectedSkill) {
            executeBtn.disabled = false;
            executeBtn.textContent = `Execute "${this.selectedSkill}" on Robot ${robotId}`;
        } else {
            executeBtn.disabled = true;
            executeBtn.textContent = teamType === 'ours' ? 'Select a skill first' : 'Cannot control opponent robots';
        }

        // Show modal
        try {
            const modalElement = document.getElementById('robotDetailModal');
            if (modalElement) {
                const modal = new bootstrap.Modal(modalElement);
                modal.show();
                console.log('Modal should be shown now');
            } else {
                console.error('Modal element not found');
            }
        } catch (error) {
            console.error('Error showing modal:', error);
            // Fallback: try using jQuery-style modal if Bootstrap 5 fails
            $('#robotDetailModal').modal('show');
        }
    }

    executeSkillOnSelectedRobot() {
        if (this.selectedRobotId !== null && this.selectedSkill) {
            // Update robot ID selector and execute
            document.getElementById('robotId').value = this.selectedRobotId;
            this.executeSelectedSkill();

            // Close modal
            try {
                const modal = bootstrap.Modal.getInstance(document.getElementById('robotDetailModal'));
                if (modal) {
                    modal.hide();
                } else {
                    // Fallback
                    $('#robotDetailModal').modal('hide');
                }
            } catch (error) {
                console.error('Error hiding modal:', error);
            }
        }
    }
}

// Initialize the application when the page loads
let craneDebugger;
document.addEventListener('DOMContentLoaded', () => {
    craneDebugger = new CraneDebugger();
});

// Global function for testing
function testShowRobotDetails(robotId) {
    console.log('testShowRobotDetails called with:', robotId);
    if (craneDebugger) {
        craneDebugger.showRobotDetails(robotId);
    } else {
        console.error('craneDebugger not initialized');
    }
}
