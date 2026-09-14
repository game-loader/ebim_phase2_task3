# Hamburg deployment confirmations

Thank you for the live interface details and attachments. We have incorporated
Fast DDS UDP-only transport, 20 Hz RELIABLE/VOLATILE arm commands, automatic
impedance configuration/activation, and a resident ROS gateway so mission phase
processes do not create new DDS participants. We use your `current_pose` directly
as TCP in the corresponding arm's link0; no additional tool offset is applied.

Both operational questions below have been answered by the organizer:

- All arm, gripper and base command topics have publisher count zero; no pause
  or mux interface is needed. The startup ownership check remains enabled.
- Both `configure_controller` services are available and permitted on domain
  0. The organizer also prepares both controllers as configured/inactive and
  ensures the arms are out of Move mode before `check`.

The organizer subsequently confirmed that GELLO commands are relative with
directions `[-1, -1, 1, 1, 1, 1, -1]` and references captured at activation.
The relay now encodes robot-space targets through that mapping; see
[HAMBURG.md](HAMBURG.md) for the activation handshake.

Original questions, retained for context:

1. **Pausing existing command publishers**

   With impedance deactivated and the arms out of Move mode, can you stop
   the existing publishers on
   `/{left,right}/gello/joint_states`,
   `/{left,right}/gripper/gripper_client/target_gripper_width_percent`, and
   `/swerve_drive_controller/cmd_vel`, while keeping the drivers and state
   broadcasters running? Alternatively, please provide the exact pause/resume
   or command-multiplexer interface we should call. Our startup currently
   refuses to take command ownership while any of these topics has an existing
   publisher, even if that publisher is temporarily silent. If pausing retains
   its DDS publisher, please provide an explicit ownership/handoff signal so
   we can adapt this check.

   Please start our container while the arms are out of Move mode, so its
   initial DDS discovery completes before activation.

2. **Configuring an unconfigured impedance controller**

   Are `/{left,right}/controller_manager/configure_controller`
   (`controller_manager_msgs/srv/ConfigureController`) available and permitted
   for our container? We call them only if `joint_impedance_controller` is
   `unconfigured`. If not, can you guarantee both impedance controllers are
   pre-configured as `inactive` before handoff?
