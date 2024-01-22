    //loop {
    //defmt::info!("Waiting for BTN0 press to start motors. Hold BTN1 for open loop, don't for velocity loop");
    //
    //while btn0.is_high() {}
    //while btn0.is_low() {}
    //
    ////if btn1.is_low(){
    ////    defmt::info!("OPEN LOOP");
    ////    control.front_left_motor.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
    ////    control.back_left_motor.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
    ////    control.back_right_motor.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
    ////    control.front_right_motor.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
    ////}
    ////else {
    ////    defmt::info!("VELOCITY LOOP");
    //control.front_left_motor.set_motion_type(MotorCommand_MotionType::VELOCITY);
    //control.back_left_motor.set_motion_type(MotorCommand_MotionType::VELOCITY);
    //control.back_right_motor.set_motion_type(MotorCommand_MotionType::VELOCITY);
    //control.front_right_motor.set_motion_type(MotorCommand_MotionType::VELOCITY);
    ////}

    //let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));
    //let mut bigger_loop_ticker = Ticker::every(Duration::from_millis(100));
    //let mut loop_duration_counter = 0.0;

    //defmt::info!("Motors starting. Press BTN0 to stop");
    //loop {
    //    if btn1.is_low() {
    //        robot_ang_vel = robot_ang_vel + 0.1;
    //    }
    //    else {
    //        robot_ang_vel = robot_ang_vel - 0.1;
    //    }

    //    loop_duration_counter = 0.0;

    //    loop {
    //        main_loop_rate_ticker.next().await;
    //        control.tick(robot_ang_vel, 0.);
    //        let err_fl = control.front_left_motor.read_is_error();
    //        let err_bl = control.back_left_motor.read_is_error();
    //        let err_br = control.back_right_motor.read_is_error();
    //        let err_fr = control.front_right_motor.read_is_error();
    //        let err_drib = control.drib_motor.read_is_error();

    //        let rpm_fl = control.front_left_motor.read_rpm();
    //        let rpm_bl = control.back_left_motor.read_rpm();
    //        let rpm_br = control.back_right_motor.read_rpm();
    //        let rpm_fr = control.front_right_motor.read_rpm();

    //        let rads_raw_fl = enc_delta_to_rads(control.front_left_motor.read_encoder_delta());
    //        let rads_raw_bl = enc_delta_to_rads(control.back_left_motor.read_encoder_delta());
    //        let rads_raw_br = enc_delta_to_rads(control.back_right_motor.read_encoder_delta());
    //        let rads_raw_fr = enc_delta_to_rads(control.front_right_motor.read_encoder_delta());

    //        let rads_filt_fl = control.front_left_motor.read_rads();
    //        let rads_filt_bl = control.back_left_motor.read_rads();
    //        let rads_filt_br = control.back_right_motor.read_rads();
    //        let rads_filt_fr = control.front_right_motor.read_rads();

    //        let vel_setpoint_fl = control.front_left_motor.read_vel_setpoint();
    //        let vel_setpoint_bl = control.back_left_motor.read_vel_setpoint();
    //        let vel_setpoint_br = control.back_right_motor.read_vel_setpoint();
    //        let vel_setpoint_fr = control.front_right_motor.read_vel_setpoint();

    //        let duty_cycle_fl = control.front_left_motor.read_vel_computed_setpoint();
    //        let duty_cycle_bl = control.back_left_motor.read_vel_computed_setpoint();
    //        let duty_cycle_br = control.back_right_motor.read_vel_computed_setpoint();
    //        let duty_cycle_fr = control.front_right_motor.read_vel_computed_setpoint();

    //        let rpm_ok_fl = within_percent_err(EXPECTED_RPM_NO_LOAD[0], rpm_fl, EXPECTED_RPM_TOLERANCE);
    //        let rpm_ok_bl = within_percent_err(EXPECTED_RPM_NO_LOAD[1], rpm_bl, EXPECTED_RPM_TOLERANCE);
    //        let rpm_ok_br = within_percent_err(EXPECTED_RPM_NO_LOAD[2], rpm_br, EXPECTED_RPM_TOLERANCE);
    //        let rpm_ok_fr = within_percent_err(EXPECTED_RPM_NO_LOAD[3], rpm_fr, EXPECTED_RPM_TOLERANCE);

    //        if  err_fl || err_bl || err_br || err_fr || err_drib {
    //            dotstar
    //                .write([COLOR_BLUE, COLOR_RED].iter().cloned())
    //                .unwrap();
    //        } else if !rpm_ok_fl || !rpm_ok_bl || !rpm_ok_br || !rpm_ok_fr {
    //            dotstar
    //                .write([COLOR_BLUE, COLOR_YELLOW].iter().cloned())
    //                .unwrap();
    //        } else {
    //            dotstar
    //                .write([COLOR_BLUE, COLOR_GREEN].iter().cloned())
    //                .unwrap();
    //        }

    //        if !err_fl && rpm_ok_fl {
    //            led0.set_high();
    //        } else {
    //            led0.set_low();
    //        }

    //        if !err_bl && rpm_ok_bl {
    //            led1.set_high();
    //        } else {
    //            led1.set_low();
    //        }

    //        if !err_br && rpm_ok_br {
    //            led2.set_high();
    //        } else {
    //            led2.set_low();
    //        }

    //        if !err_fr && rpm_ok_fr {
    //            led3.set_high();
    //        } else {
    //            led3.set_low();
    //        }

    //        if btn0.is_low() {
    //            while btn0.is_low() {}
    //            break;
    //        }

    //        defmt::info!("ROBOT ANG {:?} SET POINT {:?} {:?} {:?} {:?} DC {:?} {:?} {:?} {:?} RADS FILT {:?} {:?} {:?} {:?} RADS RAW {:?} {:?} {:?} {:?}", 
    //        robot_ang_vel,
    //        vel_setpoint_fl, vel_setpoint_bl, vel_setpoint_br, vel_setpoint_fr,
    //        duty_cycle_fl, duty_cycle_bl, duty_cycle_br, duty_cycle_fr,
    //        rads_filt_fl, rads_filt_bl, rads_filt_br, rads_filt_fr,
    //        rads_raw_fl, rads_raw_bl, rads_raw_br, rads_raw_fr);

    //        loop_duration_counter += 0.01;
    //        if loop_duration_counter > 0.5 {
    //            break;
    //        }
    //    }
    //}