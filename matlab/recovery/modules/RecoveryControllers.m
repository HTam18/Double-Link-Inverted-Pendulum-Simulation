classdef RecoveryControllers
    methods (Static)
        function metrics = attachCaseTrajectoryExport(result_dir, sim, metrics, case_index, case_type, target_name, event, cfg)
            metrics.trajectory_file = '';
            metrics.animation_available = 0;
            if ~RecoveryUtils.getLogical(cfg, 'recovery_export_all_case_trajectories', true)
                return;
            end
            try
                metrics.trajectory_file = RecoveryControllers.saveRecoveryCaseTrajectory(result_dir, sim, metrics, case_index, case_type, target_name, event, cfg);
                metrics.animation_available = 1;
            catch ME
                warning('runRecoveryMatrixCore:CaseTrajectoryExportFailed', ...
                        'Could not export trajectory for case %d (%s/%s/%s): %s', ...
                        double(case_index), char(case_type), char(target_name), char(event.name), ME.message);
                metrics.trajectory_file = '';
                metrics.animation_available = 0;
            end
        end

        function traj = buildRoaTvlqrReference(anchor_state, target_name, event, params, cfg, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            anchor_state = double(anchor_state(:));
            [T, ~, ~, ~, ~, ~] = RecoveryVariants.variantParameters(variant, cfg);
            if strcmp(char(variant), 'roa_traj_tvlqr_safe')
                T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_safe_duration_s', 4.8);
            elseif strcmp(char(variant), 'roa_traj_tvlqr_slow')
                T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_slow_duration_s', 6.0);
            elseif strcmp(char(variant), 'roa_traj_tvlqr_fast')
                T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_fast_duration_s', 3.2);
            else
                T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_nominal_duration_s', 4.2);
            end
            dt = max(RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_dt_s', 0.04), 0.01);
            time_s = (0:dt:T).';
            if time_s(end) < T
                time_s(end+1,1) = T;
            end
            n = numel(time_s);
            states = zeros(n, 6);
            u_ff = zeros(n, 1);
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            terminal_cart = min(RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_terminal_cart_abs_m', 0.35), RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 1.80));
            x_target = min(max(target_state(1), -terminal_cart), terminal_cart);

            for k = 1:n
                tau = min(max(time_s(k) / max(T, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                ds_dt = (30*tau^2 - 60*tau^3 + 30*tau^4) / max(T, eps);
                states(k,1) = (1-s) * anchor_state(1) + s * x_target;
                states(k,1) = min(max(states(k,1), -0.92 * rail), 0.92 * rail);
                states(k,2) = (1-s) * anchor_state(2) + ds_dt * (x_target - anchor_state(1));
                d1 = RecoveryUtils.wrapToPi(anchor_state(3) - target_state(3));
                d2 = RecoveryUtils.wrapToPi(anchor_state(5) - target_state(5));
                states(k,3) = target_state(3) + (1-s) * d1;
                states(k,4) = (1-s) * anchor_state(4) - ds_dt * d1;
                states(k,5) = target_state(5) + (1-s) * d2;
                states(k,6) = (1-s) * anchor_state(6) - ds_dt * d2;
                u_ff(k) = 0.0;
            end
            states(end,:) = target_state(:).';
            states(end,1) = x_target;
            states(end,2) = 0.0;
            states(end,4) = 0.0;
            states(end,6) = 0.0;

            traj = struct();
            traj.time_s = time_s;
            traj.states = states;
            traj.u_ff = u_ff;
            traj.target_name = char(target_name);
            traj.event_name = char(event.name);
            traj.variant = char(variant);
            traj.source = 'buildRoaTvlqrReference';
            tvcfg = struct();
            tvcfg.Q = diag(RecoveryUtils.getVector(cfg, 'recovery_roa_tvlqr_Q_diag', [25 6 280 25 280 25]));
            tvcfg.R = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_R', 2.5);
            tvcfg.Qf = diag(RecoveryUtils.getVector(cfg, 'recovery_roa_tvlqr_Qf_diag', [220 45 2500 180 2500 180]));
            try
                traj = designTvlqrRecoveryTrajectory(traj, params, tvcfg);
            catch ME
                warning('runRecoveryMatrixCore:TVLQRDesignFallback', 'TVLQR design failed for %s: %s', char(variant), ME.message);
            end
        end

        function variants = buildTakeoverVariantList(target_name, event, cfg)
            base = RecoveryUtils.getCell(cfg, 'recovery_roa_traj_variants', {'roa_traj_tvlqr_safe','roa_traj_tvlqr_slow','roa_traj_tvlqr_nominal','traj_cart_safe'});
            variants = {};
            if RecoveryUtils.getLogical(cfg, 'recovery_constrainedRecovery_enable', false)
                constrainedRecovery_variants = RecoveryVariants.buildConstrainedRecoveryVariantList(target_name, event, cfg);
                for iv = 1:numel(constrainedRecovery_variants)
                    variants{end+1} = char(constrainedRecovery_variants{iv}); %#ok<AGROW>
                end
            end
            if RecoveryUtils.getLogical(cfg, 'recovery_railBrakeFallback_enable', false)
                railBrakeFallback_variants = RecoveryVariants.buildRailBrakeFallbackVariantList(target_name, event, cfg);
                for iv = 1:numel(railBrakeFallback_variants)
                    variants{end+1} = char(railBrakeFallback_variants{iv}); %#ok<AGROW>
                end
            end
            if RecoveryUtils.getLogical(cfg, 'recovery_balanceFallback_enable', false)
                balanceFallback_variants = RecoveryVariants.buildBalanceFallbackVariantList(target_name, event, cfg);
                for iv = 1:numel(balanceFallback_variants)
                    variants{end+1} = char(balanceFallback_variants{iv}); %#ok<AGROW>
                end
            end
            if RecoveryControllers.recoveryIsHardStateLibraryOptimizerMode(cfg) || RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg)
                lib_variants = RecoveryVariants.buildHardStateLibraryVariants(target_name, event, cfg);
                for iv = 1:numel(lib_variants)
                    variants{end+1} = char(lib_variants{iv}); %#ok<AGROW>
                end
            end
            if RecoveryVariants.isPriority1nFailState(target_name, event, cfg)
                if (RecoveryUtils.recoveryIsUpupForceAutotuneMode(cfg) || RecoveryControllers.recoveryIsHardStateLibraryOptimizerMode(cfg) || RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg)) && strcmp(char(target_name), 'up_up')
                    if double(event.link_id) == 2 && double(event.contact_ratio) >= 1.0 - 1e-12
                        variants = [variants, {'upup_force_l2_ratio100_autotune','upup_force_l2_ratio100_railtiming','upup_force_l2_ratio100_ratekill','upup_timing_l2_ratio100','upup_timing_energy_sweep','upup_timing_after_switch'}];
                    elseif double(event.link_id) == 2 && double(event.contact_ratio) >= RecoveryUtils.getDouble(cfg, 'recovery_failstate_ratio_gate', 0.75)
                        variants = [variants, {'upup_force_l2_ratio075_autotune','upup_force_l2_ratio075_railtiming','upup_force_l2_ratio075_ratekill','upup_timing_l2_ratio075','upup_timing_energy_sweep','upup_timing_after_switch'}];
                    elseif double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12
                        variants = [variants, {'upup_force_l1_ratio100_autotune','upup_force_l1_ratio100_ratekill','upup_force_l1_ratio100_latehandoff','upup_timing_l1_ratio100','upup_timing_energy_sweep','upup_timing_after_switch'}];
                    end
                    variants = [variants, {'upup_force_after_switch_autotune','upup_timing_rail_guard','hard_catch_upup_l2_high_contact','hard_catch_upup_l1_ratio1','hard_catch_energy_angle','roa_traj_tvlqr_nominal','traj_cart_safe','terminal_upup_l2_high_contact','terminal_upup_l1_ratio1','emergency_rail_safe_takeover'}];
                elseif strcmp(char(target_name), 'up_up') && double(event.link_id) == 2 && double(event.contact_ratio) >= RecoveryUtils.getDouble(cfg, 'recovery_failstate_ratio_gate', 0.75)
                    if double(event.contact_ratio) >= 1.0 - 1e-12
                        variants{end+1} = 'upup_timing_l2_ratio100'; %#ok<AGROW>
                    else
                        variants{end+1} = 'upup_timing_l2_ratio075'; %#ok<AGROW>
                    end
                    variants{end+1} = 'upup_timing_energy_sweep'; %#ok<AGROW>
                    variants{end+1} = 'upup_timing_after_switch'; %#ok<AGROW>
                    variants{end+1} = 'upup_timing_rail_guard'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_upup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_energy_angle'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_rail_barrier'; %#ok<AGROW>
                    variants{end+1} = 'terminal_upup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'terminal_rail_brake_angle_damp_settle'; %#ok<AGROW>
                    variants{end+1} = 'terminal_low_saturation_settle'; %#ok<AGROW>
                    variants{end+1} = 'fail_upup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'emergency_rail_safe_takeover'; %#ok<AGROW>
                elseif strcmp(char(target_name), 'down_up') && double(event.link_id) == 2 && double(event.contact_ratio) >= RecoveryUtils.getDouble(cfg, 'recovery_failstate_ratio_gate', 0.75)
                    variants{end+1} = 'hard_catch_downup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_rail_barrier'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_energy_angle'; %#ok<AGROW>
                    variants{end+1} = 'terminal_downup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'terminal_rail_brake_angle_damp_settle'; %#ok<AGROW>
                    variants{end+1} = 'terminal_low_saturation_settle'; %#ok<AGROW>
                    variants{end+1} = 'fail_downup_l2_high_contact'; %#ok<AGROW>
                    variants{end+1} = 'emergency_rail_safe_takeover'; %#ok<AGROW>
                elseif strcmp(char(target_name), 'up_up') && double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12
                    variants{end+1} = 'upup_timing_l1_ratio100'; %#ok<AGROW>
                    variants{end+1} = 'upup_timing_energy_sweep'; %#ok<AGROW>
                    variants{end+1} = 'upup_timing_after_switch'; %#ok<AGROW>
                    variants{end+1} = 'upup_timing_rail_guard'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_upup_l1_ratio1'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_energy_angle'; %#ok<AGROW>
                    variants{end+1} = 'hard_catch_rail_barrier'; %#ok<AGROW>
                    variants{end+1} = 'terminal_upup_l1_ratio1'; %#ok<AGROW>
                    variants{end+1} = 'terminal_rail_brake_angle_damp_settle'; %#ok<AGROW>
                    variants{end+1} = 'terminal_long_settle'; %#ok<AGROW>
                    variants{end+1} = 'fail_upup_l1_ratio1'; %#ok<AGROW>
                    variants{end+1} = 'emergency_rail_safe_takeover'; %#ok<AGROW>
                end
            end
            for i = 1:numel(base)
                variants{end+1} = char(base{i}); %#ok<AGROW>
            end
            variants = unique(variants, 'stable');
        end

        function traj = cachedRoaTvlqrTraj(anchor_state, target_name, event, params, cfg, variant)
            persistent cache;
            if isempty(cache)
                cache = struct('key', {{}}, 'traj', {{}});
            end
            key = RecoveryControllers.roaTrajCacheKey(anchor_state, target_name, event, cfg, variant);
            for i = 1:numel(cache.key)
                if strcmp(cache.key{i}, key)
                    traj = cache.traj{i};
                    return;
                end
            end
            traj = RecoveryControllers.buildRoaTvlqrReference(anchor_state, target_name, event, params, cfg, variant);
            cache.key{end+1} = key;
            cache.traj{end+1} = traj;
        end

        function ok = candidateSatisfiesHardConstraints(m, cfg)
            ok = logical(m.pass_finite) && logical(m.pass_force_applied) && ...
                 double(m.max_abs_x_m) <= RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95) && ...
                 double(m.final_cart_abs_m) <= RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 1.80) && ...
                 double(m.saturation_fraction) <= RecoveryUtils.getDouble(cfg, 'max_saturation_fraction', 0.70);
        end

        function label = candidateValidationLabel(m, cfg)
            parts = {};
            if logical(m.success), parts{end+1} = 'PASS'; else, parts{end+1} = 'FAIL'; end %#ok<AGROW>
            if double(m.max_abs_x_m) <= RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95), parts{end+1} = 'pass_rail'; else, parts{end+1} = 'fail_rail'; end %#ok<AGROW>
            if double(m.final_angle_error_rad) <= RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10), parts{end+1} = 'pass_angle'; else, parts{end+1} = 'fail_angle'; end %#ok<AGROW>
            if double(m.final_velocity_norm) <= RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90), parts{end+1} = 'pass_velocity'; else, parts{end+1} = 'fail_velocity'; end %#ok<AGROW>
            if double(m.final_cart_abs_m) <= RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 1.80), parts{end+1} = 'pass_cart'; else, parts{end+1} = 'fail_cart'; end %#ok<AGROW>
            if double(m.saturation_fraction) <= RecoveryUtils.getDouble(cfg, 'max_saturation_fraction', 0.70), parts{end+1} = 'pass_saturation'; else, parts{end+1} = 'fail_saturation'; end %#ok<AGROW>
            if logical(m.pass_recovery_time), parts{end+1} = 'pass_settled_window'; else, parts{end+1} = 'fail_settled_window'; end %#ok<AGROW>
            label = strjoin(parts, '_');
        end

        function ctrl = constrainedTerminalRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            t_rel = max(0.0, double(t) - double(anchor_time));
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            rail_safe = RecoveryUtils.getDouble(cfg, 'recovery_terminal_cart_safe_abs_m', 1.55);
            cart_return = RecoveryUtils.getDouble(cfg, 'recovery_terminal_cart_return_abs_m', 0.85);
            brake_time = RecoveryUtils.getDouble(cfg, 'recovery_terminal_rail_brake_time_s', 1.35);
            damp_time = RecoveryUtils.getDouble(cfg, 'recovery_terminal_angle_damp_time_s', 2.20);
            settle_time = RecoveryUtils.getDouble(cfg, 'recovery_terminal_settle_time_s', 3.80);
            total_time = max(brake_time + damp_time + settle_time, RecoveryUtils.getDouble(cfg, 'recovery_terminal_total_time_s', 7.35));
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);
            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

            outward = sign(x(1)) * x(2) > 0;
            rail_risk = abs(x(1)) > rail_safe || (abs(x(2)) > 0.60 && outward);

            kx_brake = 6.0; kv_brake = 8.0; kx_settle = 2.8; kv_settle = 3.4;
            angle_rate_damp = 0.72; lqr_damp_scale = RecoveryUtils.getDouble(cfg, 'recovery_terminal_damp_lqr_scale', 0.34);
            lqr_settle_scale = RecoveryUtils.getDouble(cfg, 'recovery_terminal_settle_lqr_scale', 0.58);
            lqr_final_scale = RecoveryUtils.getDouble(cfg, 'recovery_terminal_final_lqr_scale', 0.72);
            if strcmp(char(variant), 'terminal_downup_l2_high_contact')
                kx_brake = 7.5; kv_brake = 9.5; kx_settle = 3.4; kv_settle = 4.2;
                angle_rate_damp = 0.86; lqr_damp_scale = 0.28; lqr_settle_scale = 0.50;
                brake_time = brake_time + 0.25; damp_time = damp_time + 0.35; settle_time = settle_time + 0.40;
            elseif strcmp(char(variant), 'terminal_upup_l2_high_contact')
                kx_brake = 6.6; kv_brake = 8.7; kx_settle = 3.0; kv_settle = 3.8;
                angle_rate_damp = 1.02; lqr_damp_scale = 0.32; lqr_settle_scale = 0.56;
                damp_time = damp_time + 0.45; settle_time = settle_time + 0.35;
            elseif strcmp(char(variant), 'terminal_upup_l1_ratio1')
                kx_brake = 5.8; kv_brake = 8.2; kx_settle = 2.8; kv_settle = 3.6;
                angle_rate_damp = 0.95; lqr_damp_scale = 0.34; lqr_settle_scale = 0.60;
                settle_time = settle_time + 0.45;
            elseif strcmp(char(variant), 'terminal_long_settle')
                kx_brake = 5.5; kv_brake = 7.6; kx_settle = 2.5; kv_settle = 3.2;
                angle_rate_damp = 0.78; lqr_settle_scale = 0.50; lqr_final_scale = 0.62;
                brake_time = brake_time + 0.15; damp_time = damp_time + 0.60; settle_time = settle_time + 1.05;
            elseif strcmp(char(variant), 'terminal_low_saturation_settle')
                kx_brake = 4.8; kv_brake = 7.2; kx_settle = 2.1; kv_settle = 2.9;
                angle_rate_damp = 0.66; lqr_damp_scale = 0.22; lqr_settle_scale = 0.42; lqr_final_scale = 0.54;
                brake_time = brake_time + 0.30; damp_time = damp_time + 0.55; settle_time = settle_time + 0.80;
            end
            total_time = max(total_time, brake_time + damp_time + settle_time);

            rail_excess = max(0.0, abs(x(1)) - cart_return);
            u_rail = -kx_brake * rail_excess * sign(x(1)) - kv_brake * x(2);
            if abs(x(1)) > rail_safe
                u_rail = u_rail - 12.0 * (abs(x(1)) - rail_safe) * sign(x(1));
            end
            if abs(x(1)) > rail && outward
                u_rail = u_rail - 18.0 * (abs(x(1)) - rail + 0.25*abs(x(2))) * sign(x(1));
            end

            u_rate = -angle_rate_damp * (e(4) + 0.85 * e(6));
            u_lqr = -(K * e);

            if t_rel <= brake_time || rail_risk
                stage = 'rail_braking';
                u_raw = u_rail + u_rate + 0.12 * u_lqr;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_terminal_brake_soft_u_fraction', 0.82);
            elseif t_rel <= brake_time + damp_time
                stage = 'angular_damping';
                tau = (t_rel - brake_time) / max(damp_time, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                u_cart = -kx_settle * min(max(x(1), -rail_safe), rail_safe) - kv_settle * x(2);
                u_raw = 0.65*u_rail + (1-s)*u_rate + s*(lqr_damp_scale*u_lqr) + 0.55*u_cart;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_terminal_soft_u_fraction', 0.88);
            else
                stage = 'terminal_settle';
                tau = min(max((t_rel - brake_time - damp_time) / max(settle_time, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                ref = target_state;
                d1 = RecoveryUtils.wrapToPi(anchor_state(3) - target_state(3));
                d2 = RecoveryUtils.wrapToPi(anchor_state(5) - target_state(5));
                ref(1) = (1-s) * min(max(anchor_state(1), -cart_return), cart_return) + s * target_state(1);
                ref(2) = (1-s) * anchor_state(2);
                ref(3) = target_state(3) + (1-s) * d1;
                ref(4) = (1-s) * anchor_state(4);
                ref(5) = target_state(5) + (1-s) * d2;
                ref(6) = (1-s) * anchor_state(6);
                e_ref = x - ref;
                e_ref(3) = RecoveryUtils.wrapToPi(x(3) - ref(3));
                e_ref(5) = RecoveryUtils.wrapToPi(x(5) - ref(5));
                u_track = -0.38 * (K * e_ref);
                lqr_scale = (1-s)*lqr_settle_scale + s*lqr_final_scale;
                u_cart = -kx_settle * x(1) - kv_settle * x(2);
                u_raw = 0.35*u_rail + u_track + lqr_scale*u_lqr + 0.50*u_cart + 0.30*u_rate;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_terminal_soft_u_fraction', 0.88);
            end

            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, soft_frac);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            u_cmd = min(max(double(u_raw), u_min), u_max);

            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['constrained_terminal_', stage, '_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function ctrl = failstateTakeoverController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            if startsWith(char(variant), 'constrainedRecoverya2_') || startsWith(char(variant), 'constrainedRecoveryb_')
                ctrl = constrainedRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'railBrakeFallback_')
                ctrl = recoveryRailBrakeFallbackController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'balanceFallback_')
                ctrl = recoveryBalanceFallbackController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'hardStateFallback_hs')
                ctrl = recovery_hardStateFallback_constrained_recovery_controller(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'targeted_railclamp_')
                ctrl = RecoveryControllers.targetedRailclampController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'ss_hs')
                ctrl = RecoveryControllers.stateSpecificGainSearchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'lib_')
                ctrl = RecoveryControllers.hardStateLibraryOptimizerController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'upup_timing_') || startsWith(char(variant), 'upup_force_')
                ctrl = RecoveryControllers.upupTimingAlignedCatchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'hard_catch_')
                ctrl = RecoveryControllers.hardStateCatchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if startsWith(char(variant), 'terminal_')
                ctrl = RecoveryControllers.constrainedTerminalRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            t_rel = max(0.0, double(t) - double(anchor_time));
            brake_time = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_time_s', 0.90);
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            rail_soft = min(RecoveryUtils.getDouble(cfg, 'recovery_cart_priority_abs_m', 1.40), 0.72 * rail);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);

            e_target = x - target_state;
            e_target(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e_target(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

            switch char(variant)
                case 'fail_downup_l2_high_contact'
                    kx = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kx', 7.0) * 1.25;
                    kv = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kv', 7.5) * 1.35;
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_emergency_angle_damp', 0.55) * 0.90;
                    terminal_scale = 0.55;
                    T = 5.4;
                case 'fail_upup_l2_high_contact'
                    kx = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kx', 7.0) * 1.00;
                    kv = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kv', 7.5) * 1.10;
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_emergency_angle_damp', 0.55) * 1.25;
                    terminal_scale = 0.62;
                    T = 5.0;
                case 'fail_upup_l1_ratio1'
                    kx = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kx', 7.0) * 0.90;
                    kv = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kv', 7.5) * 1.00;
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_emergency_angle_damp', 0.55) * 1.15;
                    terminal_scale = 0.66;
                    T = 4.6;
                case 'traj_cart_safe_takeover'
                    kx = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kx', 7.0) * 1.05;
                    kv = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kv', 7.5) * 1.05;
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_emergency_angle_damp', 0.55) * 0.90;
                    terminal_scale = 0.58;
                    T = 5.2;
                otherwise
                    kx = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kx', 7.0);
                    kv = RecoveryUtils.getDouble(cfg, 'recovery_emergency_brake_kv', 7.5);
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_emergency_angle_damp', 0.55);
                    terminal_scale = 0.50;
                    T = 4.8;
            end

            outward = sign(x(1)) * x(2) > 0;
            rail_risk = abs(x(1)) > rail_soft || (abs(x(2)) > 0.75 && outward);
            if t_rel <= brake_time || rail_risk
                u_cart = -kx * x(1) - kv * x(2);
                if abs(x(1)) > rail_soft
                    u_cart = u_cart - sign(x(1)) * 10.0 * (abs(x(1)) - rail_soft);
                end
                u_ang_damp = -angle_damp * (e_target(4) + 0.85 * e_target(6));
                u_terminal_soft = -0.18 * (K * e_target);
                u_raw = u_cart + u_ang_damp + u_terminal_soft;
                state_label = ['emergency_rail_safe_stage_', char(variant)];
            else
                tau = min(max((t_rel - brake_time) / max(T, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                ref = target_state;
                d1 = RecoveryUtils.wrapToPi(anchor_state(3) - target_state(3));
                d2 = RecoveryUtils.wrapToPi(anchor_state(5) - target_state(5));
                ref(1) = (1-s) * anchor_state(1) + s * target_state(1);
                ref(1) = min(max(ref(1), -0.80 * rail), 0.80 * rail);
                ref(2) = (1-s) * anchor_state(2);
                ref(3) = target_state(3) + (1-s) * d1;
                ref(4) = (1-s) * anchor_state(4);
                ref(5) = target_state(5) + (1-s) * d2;
                ref(6) = (1-s) * anchor_state(6);
                e_ref = x - ref;
                e_ref(3) = RecoveryUtils.wrapToPi(x(3) - ref(3));
                e_ref(5) = RecoveryUtils.wrapToPi(x(5) - ref(5));
                u_track = -0.55 * (K * e_ref);
                u_terminal = -terminal_scale * s * (K * e_target);
                u_cart = -0.75 * kx * e_target(1) - 0.70 * kv * e_target(2);
                u_ang_damp = -angle_damp * (e_target(4) + 0.85 * e_target(6));
                u_raw = u_track + u_terminal + u_cart + u_ang_damp;
                state_label = ['failstate_trajectory_takeover_', char(variant)];
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_terminal_recovery_enable', false)
                u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, RecoveryUtils.getDouble(cfg, 'recovery_terminal_soft_u_fraction', 0.88));
            end
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = state_label;
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function ctrl = hardStateCatchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            t_rel = max(0.0, double(t) - double(anchor_time));

            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            barrier_abs = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_abs_m', 1.72);
            emergency_abs = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_emergency_abs_m', 1.88);
            guard_time = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_guard_time_s', 1.10);
            energy_time = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_energy_time_s', 2.35);
            damp_time = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_damp_time_s', 2.20);
            handoff_time = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_handoff_time_s', 3.40);
            total_time = max(guard_time + energy_time + damp_time + handoff_time, RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_total_time_s', 9.05)); %#ok<NASGU>
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);

            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));
            u_lqr = -(K * e);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs([u_min, u_max]));

            xcart = x(1); vcart = x(2);
            sx = sign(xcart); if sx == 0, sx = 1; end
            outward = sx * vcart > 0;
            barrier_kp = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_kp', 26.0);
            barrier_kd = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_kd', 10.5);
            u_barrier = 0.0;
            if abs(xcart) > barrier_abs
                u_barrier = u_barrier - sx * barrier_kp * (abs(xcart) - barrier_abs) - barrier_kd * vcart;
            end
            if abs(xcart) > emergency_abs || (abs(xcart) > barrier_abs && outward)
                u_barrier = u_barrier - sx * (0.62 * umax + 7.0 * max(0, abs(xcart)-emergency_abs) + 2.2 * abs(vcart));
            end
            if abs(xcart) >= rail - 0.025 && outward
                u_barrier = -sx * 0.98 * umax;
            end
            u_barrier = min(max(u_barrier, u_min), u_max);

            angle_gain = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_angle_gain', 0.78);
            rate_gain = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_rate_gain', 1.15);
            lqr_min = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_terminal_lqr_min', 0.36);
            lqr_max = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_terminal_lqr_max', 0.92);
            cart_center_gain = 3.2;
            cart_damp_gain = 4.6;
            if strcmp(char(variant), 'hard_catch_downup_l2_high_contact')
                angle_gain = 0.68; rate_gain = 1.05; lqr_min = 0.28; lqr_max = 0.82;
                cart_center_gain = 4.4; cart_damp_gain = 6.0;
                guard_time = guard_time + 0.25; energy_time = energy_time + 0.20;
            elseif strcmp(char(variant), 'hard_catch_upup_l2_high_contact')
                angle_gain = 0.98; rate_gain = 1.28; lqr_min = 0.34; lqr_max = 0.96;
                cart_center_gain = 3.4; cart_damp_gain = 4.8;
                energy_time = energy_time + 0.35; damp_time = damp_time + 0.20;
            elseif strcmp(char(variant), 'hard_catch_upup_l1_ratio1')
                angle_gain = 0.90; rate_gain = 1.22; lqr_min = 0.38; lqr_max = 0.94;
                cart_center_gain = 3.0; cart_damp_gain = 4.2;
                handoff_time = handoff_time + 0.35;
            elseif strcmp(char(variant), 'hard_catch_rail_barrier')
                angle_gain = 0.45; rate_gain = 0.80; lqr_min = 0.22; lqr_max = 0.72;
                cart_center_gain = 5.0; cart_damp_gain = 6.8;
                guard_time = guard_time + 0.50;
            elseif strcmp(char(variant), 'hard_catch_energy_angle')
                angle_gain = 1.10; rate_gain = 1.35; lqr_min = 0.40; lqr_max = 1.00;
                cart_center_gain = 2.7; cart_damp_gain = 3.8;
                energy_time = energy_time + 0.55;
            end

            angle_norm = norm([e(3), e(5)]);
            rate_norm = norm([e(4), e(6)]);
            catch_blend = min(1.0, (angle_norm + 0.18*rate_norm) / 2.4);
            u_angle_catch = angle_gain * catch_blend * u_lqr - rate_gain * (e(4) + 0.72 * e(6));
            u_cart = -cart_center_gain * xcart - cart_damp_gain * vcart;

            rail_risk = abs(xcart) > barrier_abs || (abs(vcart) > 0.55 && outward);
            if t_rel <= guard_time || rail_risk
                stage = 'rail_guard';
                timing_assist = 0.18 * u_angle_catch;
                if abs(u_barrier) > 0.10*umax && sign(timing_assist) ~= sign(u_barrier)
                    timing_assist = 0.05 * timing_assist;
                end
                u_raw = u_barrier + timing_assist + 0.10*u_lqr;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_guard_u_fraction', 0.98);
            elseif t_rel <= guard_time + energy_time
                stage = 'energy_angle_catch';
                tau = (t_rel - guard_time) / max(energy_time, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                u_raw = 0.45*u_barrier + (0.45+0.35*s)*u_angle_catch + (0.20+0.22*s)*u_lqr + 0.30*u_cart;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_soft_u_fraction', 0.94);
            elseif t_rel <= guard_time + energy_time + damp_time
                stage = 'velocity_damping';
                tau = (t_rel - guard_time - energy_time) / max(damp_time, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*lqr_min + s*min(0.78, lqr_max);
                u_rate_damp = -1.35 * (e(2) + 0.85*e(4) + 0.65*e(6));
                u_raw = 0.35*u_barrier + lqr_scale*u_lqr + 0.45*u_cart + u_rate_damp;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_soft_u_fraction', 0.94);
            else
                stage = 'terminal_lqr_handoff';
                tau = min(max((t_rel - guard_time - energy_time - damp_time) / max(handoff_time, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*min(0.78, lqr_max) + s*lqr_max;
                near_balanced = angle_norm <= RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10) && ...
                                norm([e(2), e(4), e(6)]) <= RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90);
                if near_balanced && abs(xcart) > barrier_abs
                    u_raw = 1.00*u_barrier + 0.35*u_cart + 0.20*lqr_scale*u_lqr;
                    stage = 'terminal_rail_barrier_near_balanced';
                else
                    u_raw = 0.25*u_barrier + lqr_scale*u_lqr + 0.35*u_cart - 0.45*(e(4)+0.70*e(6));
                end
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_soft_u_fraction', 0.94);
            end

            if abs(xcart) > emergency_abs && outward && sign(u_raw) == sx
                u_raw = min(max(u_barrier, u_min), u_max);
                stage = [stage, '_hard_barrier_override'];
            end

            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, soft_frac);
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['hard_state_catch_', stage, '_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function ctrl = hardStateLibraryOptimizerController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            t_rel = max(0.0, double(t) - double(anchor_time));
            vname = char(variant);
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            is_after = strcmp(ct, 'after_full_switching') || contains(vname, 'lib_after_');

            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs([u_min, u_max]));
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);

            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));
            u_lqr = -(K * e);

            xcart = x(1); vcart = x(2);
            sx = sign(xcart); if sx == 0, sx = 1; end
            outward = sx * vcart > 0;

            pre_t = 0.0; guard_t = 0.65; catch_t = 4.3; cancel_t = 3.0; handoff_t = 5.0;
            barrier_abs = 1.72; emergency_abs = 1.88;
            barrier_kp = 34.0; barrier_kd = 13.0;
            timing_gain = 2.4; energy_gain = 1.35; cancel_gain = 2.8;
            cart_center_gain = 1.4; cart_damp_gain = 2.8;
            lqr_start = 0.10; lqr_end = 0.92;
            soft_frac = 0.96;

            if contains(vname, 'after')
                pre_t = 0.85; catch_t = catch_t + 1.2; cancel_t = cancel_t + 0.9; handoff_t = handoff_t + 0.8;
                timing_gain = timing_gain * 1.10; cancel_gain = cancel_gain * 1.18;
                lqr_start = 0.06; lqr_end = 0.86;
            end
            if contains(vname, 'l1_r100')
                barrier_abs = 1.82; emergency_abs = 1.92;
                barrier_kp = 20.0; barrier_kd = 8.0;
                timing_gain = 3.05; energy_gain = 1.75; cancel_gain = 3.35;
                cart_center_gain = 0.65; cart_damp_gain = 1.7;
                catch_t = catch_t + 1.6; cancel_t = cancel_t + 1.0; handoff_t = handoff_t + 1.2;
                lqr_start = 0.04; lqr_end = 0.82;
            elseif contains(vname, 'l2_r075')
                barrier_abs = 1.58; emergency_abs = 1.84;
                barrier_kp = 42.0; barrier_kd = 16.0;
                timing_gain = 2.65; energy_gain = 1.55; cancel_gain = 3.05;
                cart_center_gain = 1.55; cart_damp_gain = 3.45;
                catch_t = catch_t + 0.9; cancel_t = cancel_t + 0.8;
                lqr_start = 0.08; lqr_end = 0.88;
            elseif contains(vname, 'l2_r100')
                barrier_abs = 1.55; emergency_abs = 1.82;
                barrier_kp = 46.0; barrier_kd = 18.0;
                timing_gain = 2.95; energy_gain = 1.72; cancel_gain = 3.35;
                cart_center_gain = 1.65; cart_damp_gain = 3.75;
                catch_t = catch_t + 1.25; cancel_t = cancel_t + 1.1; handoff_t = handoff_t + 0.8;
                lqr_start = 0.06; lqr_end = 0.86;
            elseif contains(vname, 'downup_l2_r100')
                barrier_abs = 1.48; emergency_abs = 1.80;
                barrier_kp = 58.0; barrier_kd = 22.0;
                timing_gain = 1.65; energy_gain = 1.05; cancel_gain = 3.70;
                cart_center_gain = 2.35; cart_damp_gain = 5.0;
                guard_t = 0.95; catch_t = catch_t + 0.45; cancel_t = cancel_t + 1.3;
                lqr_start = 0.15; lqr_end = 0.94;
            end
            if contains(vname, 'ratekill') || contains(vname, 'velocity')
                cancel_gain = cancel_gain * 1.35; catch_t = max(2.0, catch_t - 0.5); cancel_t = cancel_t + 1.2;
            end
            if contains(vname, 'railbarrier') || contains(vname, 'railangle')
                barrier_abs = min(barrier_abs, 1.52); barrier_kp = barrier_kp * 1.22; barrier_kd = barrier_kd * 1.18;
                cart_center_gain = cart_center_gain * 1.25; cart_damp_gain = cart_damp_gain * 1.25;
            end
            if contains(vname, 'longcatch') || contains(vname, 'sweep')
                catch_t = catch_t + 1.8; handoff_t = handoff_t + 1.0; timing_gain = timing_gain * 1.10;
            end

            u_barrier = 0.0;
            if abs(xcart) > barrier_abs
                u_barrier = u_barrier - sx * barrier_kp * (abs(xcart) - barrier_abs) - barrier_kd * vcart;
            end
            if abs(xcart) > emergency_abs || (abs(xcart) > barrier_abs && outward)
                u_barrier = u_barrier - sx * (0.78 * umax + 8.0 * max(0, abs(xcart)-emergency_abs) + 2.4 * abs(vcart));
            end
            if abs(xcart) >= rail - 0.015 && outward
                u_barrier = -sx * 0.998 * umax;
            end
            u_barrier = min(max(u_barrier, u_min), u_max);

            th_err = [e(3); e(5)];
            th_rate = [e(4); e(6)];
            angle_norm = norm(th_err);
            rate_norm = norm(th_rate);
            timing_measure = th_err(1)*th_rate(1) + 0.92*th_err(2)*th_rate(2) + 0.16*(th_err(1)+th_err(2));
            timing_dir = -sign(timing_measure);
            if timing_dir == 0
                timing_dir = -sign(u_lqr); if timing_dir == 0, timing_dir = 1; end
            end
            if contains(vname, '_left')
                dir_bias = -0.10;
            elseif contains(vname, '_right')
                dir_bias = 0.10;
            else
                dir_bias = 0.0;
            end
            energy_blend = min(1.0, (angle_norm + 0.12*rate_norm) / 2.4);
            u_timing = timing_dir * energy_gain * energy_blend * umax + dir_bias * umax * energy_blend;
            u_angle = timing_gain * energy_blend * u_lqr;
            u_cancel = -cancel_gain * (0.42*e(2) + 0.95*e(4) + 0.92*e(6));
            u_cart = -cart_center_gain * xcart - cart_damp_gain * vcart;

            if t_rel <= pre_t
                stage = 'pre_settle';
                u_raw = 0.38*u_barrier + 0.28*u_lqr + 0.82*u_cancel + 0.55*u_cart;
                soft_now = min(0.90, soft_frac);
            elseif t_rel <= pre_t + guard_t || (abs(xcart) > barrier_abs && outward)
                stage = 'rail_guard';
                assist = 0.16*u_angle + 0.12*u_timing;
                if abs(u_barrier) > 0.12*umax && sign(assist) ~= sign(u_barrier)
                    assist = 0.05*assist;
                end
                u_raw = u_barrier + assist + 0.05*u_lqr;
                soft_now = 0.99;
            elseif t_rel <= pre_t + guard_t + catch_t
                stage = 'timing_aligned_energy_catch';
                tau = (t_rel - pre_t - guard_t) / max(catch_t, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                u_raw = 0.16*u_barrier + (0.55+0.30*s)*u_timing + (0.64+0.30*s)*u_angle + 0.10*u_cart;
                if abs(xcart) > 0.90*barrier_abs && outward && sign(u_raw) == sx
                    u_raw = 0.88*u_barrier + 0.16*u_angle + 0.10*u_cancel;
                    stage = 'timing_catch_rail_limited';
                end
                soft_now = soft_frac;
            elseif t_rel <= pre_t + guard_t + catch_t + cancel_t
                stage = 'angular_velocity_cancellation';
                tau = (t_rel - pre_t - guard_t - catch_t) / max(cancel_t, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*lqr_start + s*min(0.72, lqr_end);
                u_raw = 0.18*u_barrier + 0.78*lqr_scale*u_lqr + 1.05*u_cancel + 0.22*u_cart;
                soft_now = soft_frac;
            else
                stage = 'terminal_lqr_handoff';
                tau = min(max((t_rel - pre_t - guard_t - catch_t - cancel_t) / max(handoff_t, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*min(0.72, lqr_end) + s*lqr_end;
                if angle_norm < 0.18 && rate_norm < 1.8 && abs(xcart) < 1.65
                    u_raw = lqr_scale*u_lqr + 0.18*u_cart + 0.26*u_cancel + 0.10*u_barrier;
                elseif angle_norm < 0.38 && rate_norm >= 1.4
                    u_raw = 0.32*u_barrier + 0.58*lqr_scale*u_lqr + 1.25*u_cancel + 0.18*u_cart;
                    stage = 'terminal_ratekill_before_lqr';
                else
                    u_raw = 0.20*u_barrier + 0.74*lqr_scale*u_lqr + 0.62*u_cancel + 0.18*u_cart;
                end
                soft_now = soft_frac;
            end

            if abs(xcart) > emergency_abs && outward && sign(u_raw) == sx
                u_raw = u_barrier;
                stage = [stage, '_hard_rail_override'];
            end
            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, soft_now);
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['hard_state_library_optimizer_', stage, '_', vname];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function score = metricRoaScore(m, cfg)
            gate_a = max(RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10), eps);
            gate_v = max(RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90), eps);
            gate_x = max(RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 0.85), eps);
            rail = max(RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.0), eps);
            wa = RecoveryUtils.getDouble(cfg, 'recovery_roa_angle_weight', 2.0);
            wv = RecoveryUtils.getDouble(cfg, 'recovery_roa_velocity_weight', 1.2);
            wx = RecoveryUtils.getDouble(cfg, 'recovery_roa_cart_weight', 1.4);
            ws = RecoveryUtils.getDouble(cfg, 'recovery_roa_saturation_weight', 2.5);
            wr = RecoveryUtils.getDouble(cfg, 'recovery_roa_rail_weight', 3.0);
            score = wa * double(m.final_angle_error_rad) / gate_a + ...
                    wv * double(m.final_velocity_norm) / gate_v + ...
                    wx * double(m.final_cart_abs_m) / gate_x + ...
                    ws * double(m.saturation_fraction) + ...
                    wr * max(0.0, double(m.max_abs_x_m) / rail - 0.75);
            if ~logical(m.pass_finite), score = score + 1.0e6; end
            if ~logical(m.pass_force_applied), score = score + 1.0e4; end
            if ~logical(m.pass_rail), score = score + 1.0e5; end
        end

        function tf = optCandidateIsSafeForSelection(m, cfg)
            tf = true;
            if ~logical(m.pass_finite) || ~logical(m.pass_force_applied)
                tf = false;
                return;
            end
            if RecoveryUtils.getLogical(cfg, 'recovery_safeOpt_reject_opt_if_rail_fail', true) && ~logical(m.pass_rail)
                tf = false;
                return;
            end
            max_sat = RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_opt_max_saturation_fraction', 0.35);
            max_final_cart = RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_opt_max_final_cart_abs_m', 0.85);
            max_peak_cart = RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_opt_max_peak_cart_abs_m', 0.95);
            max_recovery_time = RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_max_opt_recovery_time_s', 6.0);
            if double(m.saturation_fraction) > max_sat
                tf = false;
                return;
            end
            if double(m.final_cart_abs_m) > max_final_cart
                tf = false;
                return;
            end
            if double(m.max_abs_x_m) > max_peak_cart
                tf = false;
                return;
            end
            if isfinite(double(m.recovery_time_s)) && double(m.recovery_time_s) > max_recovery_time
                tf = false;
                return;
            end
        end

        function tf = recoveryIsHardStateLibraryOptimizerMode(cfg)
            tf = false;
            if ~isfield(cfg, 'recovery_mode'), return; end
            mode = upper(char(string(cfg.recovery_mode)));
            tf = ~isempty(strfind(mode, 'HARD_STATE_LIBRARY_OPTIMIZER')) || ~isempty(strfind(mode, 'hard-state fallback_1N_CONSTRAINED_HARDSTATE_RECOVERY')) || ~isempty(strfind(mode, 'BALANCE_FALLBACK_1N_TWO_BRANCH_RESCUE')) || ~isempty(strfind(mode, 'RAIL_BRAKE_FALLBACK_RAIL_ONLY_PREEMPTIVE_BRAKE')) || ~isempty(strfind(mode, 'CONSTRAINED_RECOVERY_CONSTRAINED_TRAJ_TVLQR_ROA_MICROBRAKE'));
        end

        function tf = recoveryIsStateSpecificGainSearchMode(cfg)
            tf = false;
            if ~isfield(cfg, 'recovery_mode'), return; end
            mode = upper(char(string(cfg.recovery_mode)));
            tf = ~isempty(strfind(mode, 'STATE_SPECIFIC_GAIN_SEARCH')) || ~isempty(strfind(mode, 'REBASE_BEST50_TARGETED_REPAIR')) || ~isempty(strfind(mode, 'hard-state fallback_1N_CONSTRAINED_HARDSTATE_RECOVERY')) || ~isempty(strfind(mode, 'BALANCE_FALLBACK_1N_TWO_BRANCH_RESCUE')) || ~isempty(strfind(mode, 'RAIL_BRAKE_FALLBACK_RAIL_ONLY_PREEMPTIVE_BRAKE')) || ~isempty(strfind(mode, 'CONSTRAINED_RECOVERY_CONSTRAINED_TRAJ_TVLQR_ROA_MICROBRAKE'));
        end

        function key = roaTrajCacheKey(anchor_state, target_name, event, cfg, variant)
            a = round(double(anchor_state(:)).' * 1000) / 1000;
            key = sprintf('%s|%s|L%d|R%.2f|%s|F%.2f|%s', char(target_name), char(variant), double(event.link_id), double(event.contact_ratio), char(event.direction_name), double(event.force_N), mat2str(a,4));
        end

        function ctrl = roaTvlqrRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            traj = RecoveryControllers.cachedRoaTvlqrTraj(anchor_state, target_name, event, params, cfg, variant);
            t_rel = max(0.0, double(t) - double(anchor_time));
            if t_rel <= double(traj.time_s(end))
                ctrl = tvlqrRecoveryTracking(t_rel, x, traj, params, cfg);
                ctrl.recovery_state = ['roa_traj_tvlqr_tracking_', char(variant)];
                ctrl.mode = ctrl.recovery_state;
            else
                ctrl = disturbanceRecoveryController(t, x, params, cfg);
                ctrl.recovery_state = ['roa_traj_tvlqr_terminal_lqr_', char(variant)];
                ctrl.mode = ctrl.recovery_state;
            end
        end

        function trajectory_file = saveRecoveryCaseTrajectory(result_dir, sim, metrics, case_index, case_type, target_name, event, cfg)
            traj_dir = fullfile(result_dir, char(cfg.recovery_case_trajectory_dir_name));
            if ~exist(traj_dir, 'dir'), mkdir(traj_dir); end

            safe_case_type = RecoveryIo.safeFileToken(case_type);
            safe_target = RecoveryIo.safeFileToken(target_name);
            safe_direction = RecoveryIo.safeFileToken(event.direction_name);
            ratio_tag = sprintf('ratio%03d', round(double(event.contact_ratio) * 100));
            force_tag = sprintf('%gN', double(event.force_N));
            file_name = sprintf('%04d_%s_%s_link%d_%s_%s_%s.mat', ...
                double(case_index), safe_case_type, safe_target, double(event.link_id), ratio_tag, safe_direction, force_tag);
            trajectory_file = fullfile(traj_dir, file_name);

            case_info = struct();
            case_info.case_index = double(case_index);
            case_info.case_type = char(case_type);
            case_info.target = char(target_name);
            case_info.event_name = char(event.name);
            case_info.link_id = double(event.link_id);
            case_info.contact_ratio = double(event.contact_ratio);
            case_info.direction = char(event.direction_name);
            case_info.force_N = double(event.force_N);
            case_info.duration_s = double(event.duration_s);
            case_info.start_time_s = double(event.start_time_s);
            case_info.end_time_s = double(event.end_time_s);

            trajectory = struct();
            trajectory.time_s = RecoveryIo.forceCol(sim.time_s);
            trajectory.states = double(sim.states);
            trajectory.u_cmd = RecoveryIo.forceCol(sim.u_cmd);
            if isfield(sim, 'u_raw'), trajectory.u_raw = RecoveryIo.forceCol(sim.u_raw); end
            if isfield(sim, 'u_actual'), trajectory.u_actual = RecoveryIo.forceCol(sim.u_actual); end
            if isfield(sim, 'q_external'), trajectory.q_external = double(sim.q_external); end
            if isfield(sim, 'force_x_N'), trajectory.force_x_N = RecoveryIo.forceCol(sim.force_x_N); end
            if isfield(sim, 'force_y_N'), trajectory.force_y_N = RecoveryIo.forceCol(sim.force_y_N); end
            if isfield(sim, 'active'), trajectory.active = logical(sim.active(:)); end
            if isfield(sim, 'recovery_state'), trajectory.recovery_state = sim.recovery_state; end
            if isfield(sim, 'controller_variant'), trajectory.controller_variant = sim.controller_variant; end

            case_metrics = metrics;
            save(trajectory_file, 'trajectory', 'case_info', 'case_metrics', '-v7.3');
        end

        function traj = selectOptimizedRecoveryTraj(anchor_state, target_name, event, cfg)
            traj = [];
            if ~RecoveryUtils.getLogical(cfg, 'recovery_enable_optimized_library', true)
                return;
            end
            project_root = RecoveryUtils.projectRoot();
            lib_path = fullfile(project_root, 'shared', 'recovery_trajectories_opt', 'recovery_recovery_opt_library.mat');
            if ~isfile(lib_path), return; end
            try
                S = load(lib_path);
                if ~isfield(S, 'library') || ~isfield(S.library, 'entries') || isempty(S.library.entries), return; end
                entries = S.library.entries;
                best_i = 0; best_d = Inf;
                x = double(anchor_state(:));
                allow_near = RecoveryUtils.getLogical(cfg, 'recovery_opt_allow_near_pass_route', false);
                max_d_pass = RecoveryUtils.getDouble(cfg, 'recovery_opt_max_route_distance_pass', RecoveryUtils.getDouble(cfg, 'recovery_opt_max_route_distance', 3.0));
                max_d_near = RecoveryUtils.getDouble(cfg, 'recovery_opt_max_route_distance_near', 2.60);
                force_tol_pass = RecoveryUtils.getDouble(cfg, 'recovery_opt_force_tolerance_pass_N', 1.25);
                force_tol_near = RecoveryUtils.getDouble(cfg, 'recovery_opt_force_tolerance_near_N', 1.10);
                for i = 1:numel(entries)
                    if ~strcmp(char(entries(i).target), char(target_name)), continue; end
                    has_pass = isfield(entries, 'validation_pass') && logical(entries(i).validation_pass);
                    has_near = isfield(entries, 'validation_near_pass') && logical(entries(i).validation_near_pass);
                    if ~has_pass
                        if ~(allow_near && has_near)
                            continue;
                        end
                        if ~(abs(double(event.force_N) - 2.0) < 1e-9 || abs(double(event.force_N) - 3.0) < 1e-9)
                            continue;
                        end
                    end
                    if ~isfield(entries, 'path') || isempty(entries(i).path) || ~isfile(entries(i).path), continue; end
                    force_diff = abs(double(entries(i).force_N) - double(event.force_N));
                    if has_pass
                        if force_diff > force_tol_pass, continue; end
                        route_limit = max_d_pass;
                    else
                        if force_diff > force_tol_near, continue; end
                        route_limit = max_d_near;
                    end
                    event_penalty = 0;
                    event_penalty = event_penalty + 1.0 * (double(entries(i).link_id) ~= double(event.link_id));
                    event_penalty = event_penalty + 0.7 * force_diff;
                    event_penalty = event_penalty + 0.9 * abs(double(entries(i).contact_ratio) - double(event.contact_ratio));
                    if ~strcmp(char(entries(i).direction), char(event.direction_name)), event_penalty = event_penalty + 1.25; end
                    if ~has_pass
                        event_penalty = event_penalty + 0.60;
                    end
                    dx = RecoveryMetrics.weightedStateDistance(x, double(entries(i).x_start(:)));
                    d = dx + event_penalty;
                    if d <= route_limit && d < best_d
                        best_d = d; best_i = i;
                    end
                end
                if best_i > 0
                    T = load(entries(best_i).path);
                    if isfield(T, 'traj')
                        traj = T.traj;
                        traj.route_distance = best_d;
                        traj.route_entry = entries(best_i);
                        traj.route_policy = 'safe optimization_SAFE_OPT_SELECTION_penalty_route_final_selection_safety_gated';
                    end
                end
            catch ME
                warning('runRecoveryMatrixCore:OptLibraryRouteFailed', 'Could not route optimized recovery trajectory: %s', ME.message);
                traj = [];
            end
        end

        function choose_opt = shouldSelectOptCandidate(m_local, cost_local, m_opt, cost_opt, opt_used, improve_margin, cfg)
            choose_opt = false;
            if ~opt_used || isempty(m_opt) || ~isfinite(cost_opt)
                return;
            end

            safe_opt = RecoveryControllers.optCandidateIsSafeForSelection(m_opt, cfg);
            if ~safe_opt
                return;
            end

            require_success = RecoveryUtils.getLogical(cfg, 'recovery_safeOpt_require_opt_success_for_selection', true);
            allow_safe_failed = RecoveryUtils.getLogical(cfg, 'recovery_safeOpt_allow_safe_failed_opt_selection', false);
            if require_success && ~logical(m_opt.success)
                return;
            end
            if ~logical(m_opt.success) && ~allow_safe_failed
                return;
            end

            roa_local = RecoveryControllers.metricRoaScore(m_local, cfg);
            roa_opt = RecoveryControllers.metricRoaScore(m_opt, cfg);
            min_roa_improve = RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_min_roa_improvement', 0.35);
            min_cost_improve = max(RecoveryUtils.getDouble(cfg, 'recovery_safeOpt_min_cost_improvement', 50.0), double(improve_margin));
            opt_reduces_roa = isfinite(roa_local) && isfinite(roa_opt) && ((roa_local - roa_opt) >= min_roa_improve);
            opt_reduces_cost = isfinite(cost_local) && isfinite(cost_opt) && ((cost_local - cost_opt) >= min_cost_improve);

            if logical(m_opt.success) && ~logical(m_local.success) && RecoveryUtils.getLogical(cfg, 'recovery_roaRouter_select_if_opt_success_and_local_fail', true)
                choose_opt = true;
                return;
            end

            if logical(m_opt.success) && logical(m_local.success)
                sat_ok = double(m_opt.saturation_fraction) <= double(m_local.saturation_fraction) + 0.05;
                time_ok = ~isfinite(double(m_local.recovery_time_s)) || double(m_opt.recovery_time_s) <= double(m_local.recovery_time_s) + 0.15;
                if (opt_reduces_roa || opt_reduces_cost) && sat_ok && time_ok
                    choose_opt = true;
                end
                return;
            end

            if allow_safe_failed && ~logical(m_opt.success) && ~logical(m_local.success)
                if opt_reduces_roa && opt_reduces_cost
                    choose_opt = true;
                end
            end
        end

        function choose = shouldSelectRoaTrajCandidate(m_best, cost_best, m_try, cost_try, cfg)
            choose = false;
            if isempty(m_try) || ~isfinite(cost_try)
                return;
            end
            if ~RecoveryControllers.candidateSatisfiesHardConstraints(m_try, cfg)
                return;
            end
            if logical(m_try.success) && ~logical(m_best.success)
                choose = true;
                return;
            end
            if logical(m_try.success) && logical(m_best.success)
                choose = cost_try <= cost_best - RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_min_cost_improvement', 10.0);
                return;
            end
            if ~logical(m_best.success) && RecoveryUtils.getLogical(cfg, 'recovery_roa_traj_allow_safe_improved_failed_candidate', false)
                roa_best = RecoveryControllers.metricRoaScore(m_best, cfg);
                roa_try = RecoveryControllers.metricRoaScore(m_try, cfg);
                choose = isfinite(roa_best) && isfinite(roa_try) && roa_try <= roa_best - RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_min_roa_improvement', 0.50);
            end
        end

        function [try_opt, reason] = shouldTryOptCandidate(m_local, target_name, event, cfg)
            try_opt = false;
            reasons = {};
            if ~RecoveryUtils.getLogical(cfg, 'recovery_roaRouter_enable_roa_router', true)
                reason = 'ROA_ROUTER_DISABLED';
                return;
            end
            priority_targets = RecoveryUtils.getCell(cfg, 'recovery_roaRouter_priority_targets', {'up_up','down_up'});
            nominal_forces = RecoveryUtils.getVector(cfg, 'recovery_roaRouter_nominal_opt_forces_N', [2 3]);
            is_priority_target = any(strcmp(char(target_name), priority_targets));
            is_nominal_force = any(abs(double(event.force_N) - nominal_forces) < 1e-9);
            high_sat = double(m_local.saturation_fraction) >= RecoveryUtils.getDouble(cfg, 'recovery_roaRouter_local_high_saturation_fraction', 0.42);
            slow_recovery = (~isfinite(double(m_local.recovery_time_s))) || double(m_local.recovery_time_s) >= RecoveryUtils.getDouble(cfg, 'recovery_roaRouter_local_slow_recovery_s', 2.20);
            high_roa = RecoveryControllers.metricRoaScore(m_local, cfg) >= RecoveryUtils.getDouble(cfg, 'recovery_roaRouter_local_high_roa_score', 7.50);
            near_pass = RecoveryMetrics.metricsNearPass(m_local, cfg);

            if ~logical(m_local.success)
                try_opt = true; reasons{end+1} = 'LOCAL_FAIL'; %#ok<AGROW>
            end
            if is_priority_target && is_nominal_force
                try_opt = true; reasons{end+1} = 'PRIORITY_TARGET_NOMINAL_FORCE'; %#ok<AGROW>
            end
            if high_sat
                try_opt = true; reasons{end+1} = 'LOCAL_HIGH_SATURATION'; %#ok<AGROW>
            end
            if slow_recovery
                try_opt = true; reasons{end+1} = 'LOCAL_SLOW_OR_NO_RECOVERY'; %#ok<AGROW>
            end
            if high_roa
                try_opt = true; reasons{end+1} = 'LOCAL_HIGH_ROA_SCORE'; %#ok<AGROW>
            end
            if near_pass && is_priority_target && is_nominal_force
                try_opt = true; reasons{end+1} = 'NEAR_PASS_BUT_TARGET_IS_KNOWN_WEAK'; %#ok<AGROW>
            end
            if isempty(reasons)
                reason = 'LOCAL_GOOD_NO_OPT_NEEDED';
            else
                reason = strjoin(unique(reasons, 'stable'), '|');
            end
        end

        function [try_traj, reason] = shouldTryRoaTrajCandidate(m_local, target_name, event, cfg)
            reasons = {};
            priority_targets = RecoveryUtils.getCell(cfg, 'recovery_roaRouter_priority_targets', {'up_up','down_up'});
            priority_contact = double(event.contact_ratio) >= RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_priority_contact_ratio', 0.75);
            priority_link = double(event.link_id) == RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_priority_link_id', 2);
            is_priority_target = any(strcmp(char(target_name), priority_targets));
            local_roa_score = RecoveryControllers.metricRoaScore(m_local, cfg);
            high_roa = isfinite(local_roa_score) && local_roa_score >= RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_try_roa_score', 3.50);
            high_sat = double(m_local.saturation_fraction) >= RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_try_saturation_fraction', 0.30);
            near_rail = double(m_local.max_abs_x_m) >= RecoveryUtils.getDouble(cfg, 'recovery_roa_traj_try_peak_cart_abs_m', 1.45);
            no_settle = ~logical(m_local.pass_recovery_time);
            if ~logical(m_local.success), reasons{end+1} = 'LOCAL_FAIL'; end %#ok<AGROW>
            if is_priority_target && (priority_link || priority_contact), reasons{end+1} = 'WEAK_TARGET_LINK_OR_HIGH_CONTACT'; end %#ok<AGROW>
            if high_roa, reasons{end+1} = 'LOCAL_HIGH_ROA_SCORE'; end %#ok<AGROW>
            if high_sat, reasons{end+1} = 'LOCAL_HIGH_SATURATION'; end %#ok<AGROW>
            if near_rail, reasons{end+1} = 'LOCAL_NEAR_OR_OVER_RAIL'; end %#ok<AGROW>
            if no_settle, reasons{end+1} = 'LOCAL_NO_SETTLED_WINDOW'; end %#ok<AGROW>
            try_traj = ~isempty(reasons);
            if try_traj
                reason = strjoin(unique(reasons, 'stable'), '|');
            else
                reason = 'LOCAL_INSIDE_ROA_NO_TRAJ_NEEDED';
            end
        end

        function tf = simUsedOptTrajectory(sim)
            tf = false;
            if ~isfield(sim, 'recovery_state') || isempty(sim.recovery_state)
                return;
            end
            for ii = 1:numel(sim.recovery_state)
                state_i = sim.recovery_state{ii};
                if isempty(state_i), continue; end
                if contains(char(state_i), 'optimized_library_recovery_tracking') || contains(char(state_i), 'optimized_library_post_trajectory_local_settle')
                    tf = true;
                    return;
                end
            end
        end

        function ctrl = stateSpecificGainSearchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            [hs, gid, suffix] = RecoveryVariants.parseStateSpecificVariant(variant);
            if isempty(hs)
                ctrl = disturbanceRecoveryController(t, x, params, cfg);
                ctrl.recovery_state = 'state_specific_parse_failed_fallback_local';
                return;
            end
            gp = RecoveryVariants.stateSpecificGainParams(hs, gid, suffix);
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:)); %#ok<NASGU>
            t_rel = max(0.0, double(t) - double(anchor_time));
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs([u_min, u_max]));
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);
            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));
            u_lqr = -(K * e);
            xcart = x(1); vcart = x(2);
            sx = sign(xcart); if sx == 0, sx = 1; end
            outward = sx * vcart > 0;
            angle_norm = norm([e(3), e(5)]);
            vel_norm = norm([e(2), e(4), e(6)]);
            near_balanced = angle_norm <= 0.16 && vel_norm <= 1.25;
            u_barrier = 0.0;
            if abs(xcart) > gp.barrier_abs
                u_barrier = u_barrier - sx * gp.barrier_kp * (abs(xcart)-gp.barrier_abs) - gp.barrier_kd * vcart;
            end
            if abs(xcart) > gp.emergency_abs || (abs(xcart) > gp.barrier_abs && outward)
                u_barrier = u_barrier - sx * (0.82*umax + 5.0*max(0, abs(xcart)-gp.emergency_abs) + 2.0*abs(vcart));
            end
            if abs(xcart) >= rail - 0.010 && outward
                u_barrier = -sx * 0.999 * umax;
            end
            u_barrier = min(max(u_barrier, u_min), u_max);
            th_err = [e(3); e(5)];
            th_rate = [e(4); e(6)];
            timing_measure = th_err(1)*th_rate(1) + 0.92*th_err(2)*th_rate(2) + 0.14*(th_err(1)+th_err(2));
            timing_dir = -sign(timing_measure);
            if timing_dir == 0, timing_dir = -sign(u_lqr); end
            if timing_dir == 0, timing_dir = gp.pulse_sign; end
            energy_blend = min(1.0, (angle_norm + 0.15*norm(th_rate)) / 2.2);
            u_pulse = gp.pulse_sign * gp.pulse_frac * umax * energy_blend;
            u_timing = timing_dir * gp.energy_gain * umax * energy_blend;
            u_angle = gp.timing_gain * energy_blend * u_lqr;
            u_cancel = -gp.cancel_gain * (0.25*e(2) + 0.95*e(4) + 0.95*e(6));
            u_cart = -gp.cart_gain * xcart - gp.cart_damp * vcart;
            if t_rel <= gp.pre_t
                stage = 'pre_settle';
                u_raw = 0.70*u_barrier + 0.50*u_cancel + 0.18*u_cart + 0.10*u_lqr;
            elseif (gp.rail_first && (abs(xcart) > gp.barrier_abs || (abs(xcart) > 1.35 && outward))) || ...
                   (near_balanced && (gp.near_balanced_rail_clamp || abs(xcart) > gp.barrier_abs))
                stage = 'hard_rail_clamp';
                u_raw = 1.00*u_barrier + 0.20*u_cart + 0.10*u_lqr;
            elseif t_rel <= gp.pre_t + gp.guard_t
                stage = 'guard_timing_aligned';
                if gp.angle_first && abs(xcart) < gp.barrier_abs
                    u_raw = 0.08*u_barrier + 0.52*u_timing + 0.46*u_angle + 0.16*u_pulse + 0.12*u_cart;
                else
                    u_raw = 0.78*u_barrier + 0.18*u_timing + 0.18*u_angle + 0.08*u_lqr;
                end
            elseif t_rel <= gp.pre_t + gp.guard_t + gp.catch_t
                stage = 'angle_energy_catch';
                tau = (t_rel - gp.pre_t - gp.guard_t) / max(gp.catch_t, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                u_raw = (0.18+0.08*s)*u_barrier + (0.38+0.18*s)*u_timing + (0.46+0.30*s)*u_angle + (0.22-0.12*s)*u_pulse + 0.10*u_cart;
                if abs(xcart) > 0.94*gp.barrier_abs && outward && sign(u_raw) == sx
                    u_raw = 0.90*u_barrier + 0.16*u_angle + 0.08*u_cancel;
                    stage = 'angle_energy_catch_barrier_limited';
                end
            elseif t_rel <= gp.pre_t + gp.guard_t + gp.catch_t + gp.cancel_t
                stage = 'angular_velocity_cancellation';
                tau = (t_rel - gp.pre_t - gp.guard_t - gp.catch_t) / max(gp.cancel_t, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = gp.lqr_min + (min(0.78,gp.lqr_max)-gp.lqr_min)*s;
                u_raw = 0.20*u_barrier + lqr_scale*u_lqr + 0.90*u_cancel + 0.15*u_cart;
            else
                stage = 'terminal_lqr_handoff';
                tau = min(max((t_rel - gp.pre_t - gp.guard_t - gp.catch_t - gp.cancel_t) / max(gp.handoff_t, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = min(0.80,gp.lqr_max) + (gp.lqr_max-min(0.80,gp.lqr_max))*s;
                if near_balanced && abs(xcart) > 1.70
                    u_raw = 1.00*u_barrier + 0.25*u_cart + 0.15*lqr_scale*u_lqr;
                    stage = 'terminal_rail_clamp_near_balanced';
                else
                    u_raw = 0.18*u_barrier + lqr_scale*u_lqr + 0.30*u_cancel + 0.12*u_cart;
                end
            end
            if abs(xcart) > gp.emergency_abs && outward && sign(u_raw) == sx
                u_raw = min(max(u_barrier, u_min), u_max);
                stage = [stage, '_absolute_rail_override'];
            end
            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, gp.soft_frac);
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['state_specific_gain_search_', hs, '_g', sprintf('%02d', round(gid)), '_', stage, '_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function ctrl = targetedRailclampController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            t_rel = max(0.0, double(t) - double(anchor_time)); %#ok<NASGU>
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            guard_abs = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_guard_abs_m', 1.80);
            hard_abs = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_hard_abs_m', 1.90);
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);
            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));
            u_lqr = -(K * e);
            xcart = x(1); vcart = x(2);
            sx = sign(xcart); if sx == 0, sx = 1; end
            outward = sx * vcart > 0;
            u_barrier = 0.0;
            stage = 'local_recovery_with_targeted_clamp_idle';
            if outward && abs(xcart) > guard_abs
                excess = max(0.0, abs(xcart) - guard_abs);
                kp = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_kp', 34.0);
                kd = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_kd', 12.0);
                u_barrier = -sx * (kp * excess + kd * abs(vcart));
                stage = 'targeted_rail_guard';
            end
            if outward && abs(xcart) > hard_abs
                kp_h = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_hard_kp', 58.0);
                kd_h = RecoveryUtils.getDouble(cfg, 'recovery_targeted_rail_clamp_hard_kd', 20.0);
                u_raw = -sx * min(max(abs(kp_h * (abs(xcart)-hard_abs) + kd_h * abs(vcart)), 0.55*max(abs([u_min,u_max]))), max(abs([u_min,u_max])));
                stage = 'targeted_hard_rail_override';
            elseif abs(u_barrier) > 0
                u_raw = 0.35*u_lqr + u_barrier;
            else
                u_raw = u_lqr;
            end
            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, 0.98);
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = [stage, '_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function ctrl = trajectoryRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            if RecoveryControllers.variantIsRoaTvlqr(variant)
                ctrl = RecoveryControllers.roaTvlqrRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            if RecoveryControllers.variantIsTakeoverGuard(variant)
                ctrl = RecoveryControllers.failstateTakeoverController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
                return;
            end
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            [T, track_scale, terminal_scale, cart_gain, cart_damp, angle_damp] = RecoveryVariants.variantParameters(variant, cfg);
            tau = min(max((double(t) - double(anchor_time)) / max(T, eps), 0.0), 1.0);
            s = 10*tau^3 - 15*tau^4 + 6*tau^5;

            ref = target_state;
            ref(1) = (1 - s) * anchor_state(1) + s * target_state(1);
            ref(2) = (1 - s) * anchor_state(2);
            dth1 = RecoveryUtils.wrapToPi(anchor_state(3) - target_state(3));
            dth2 = RecoveryUtils.wrapToPi(anchor_state(5) - target_state(5));
            ref(3) = target_state(3) + (1 - s) * dth1;
            ref(4) = (1 - s) * anchor_state(4);
            ref(5) = target_state(5) + (1 - s) * dth2;
            ref(6) = (1 - s) * anchor_state(6);

            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);
            e_ref = x - ref;
            e_ref(3) = RecoveryUtils.wrapToPi(x(3) - ref(3));
            e_ref(5) = RecoveryUtils.wrapToPi(x(5) - ref(5));
            e_target = x - target_state;
            e_target(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e_target(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

            terminal_blend = RecoveryUtils.getDouble(cfg, 'recovery_traj_terminal_blend_min', 0.18) + ...
                             (RecoveryUtils.getDouble(cfg, 'recovery_traj_terminal_blend_max', 0.90) - RecoveryUtils.getDouble(cfg, 'recovery_traj_terminal_blend_min', 0.18)) * s;
            u_track = -K * e_ref;
            u_terminal = -K * e_target;
            u_cart = -cart_gain * e_target(1) - cart_damp * e_target(2);
            u_damp = -angle_damp * (e_target(4) + 0.75 * e_target(6));
            u_raw = track_scale * u_track + terminal_scale * terminal_blend * u_terminal + u_cart + u_damp;

            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            u_cmd = min(max(double(u_raw), u_min), u_max);

            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['trajectory_recovery_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.reference_state = ref;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function score = upupFailedCandidateScore(m, variant, cfg)
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            angle_gate = RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10);
            vel_gate = RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90);
            sat_gate = RecoveryUtils.getDouble(cfg, 'max_saturation_fraction', 0.70);
            rail_excess = max(0, double(m.max_abs_x_m) - rail);
            angle_excess = max(0, double(m.final_angle_error_rad) - angle_gate);
            vel_excess = max(0, double(m.final_velocity_norm) - vel_gate);
            sat_excess = max(0, double(m.saturation_fraction) - sat_gate);
            settle_penalty = 0;
            if ~logical(m.pass_recovery_time), settle_penalty = 8; end
            pass_bonus = 0;
            if logical(m.success), pass_bonus = -1e5; end
            variant_bonus = 0;
            v = char(variant);
            if startsWith(v, 'ss_hs'), variant_bonus = -90; end
            if startsWith(v, 'lib_'), variant_bonus = -36; end
            if startsWith(v, 'upup_force_'), variant_bonus = -18; end
            if startsWith(v, 'upup_timing_'), variant_bonus = -8; end
            score = pass_bonus + 1600*rail_excess^2 + 55*vel_excess + 70*angle_excess + 150*sat_excess + settle_penalty + double(m.final_cart_abs_m) + variant_bonus;
        end

        function ctrl = upupTimingAlignedCatchController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
            lib = targetEquilibriumLibrary(params);
            target_state = lib.targets.(target_name).target_state(:);
            x = double(x(:));
            anchor_state = double(anchor_state(:));
            t_rel = max(0.0, double(t) - double(anchor_time));

            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            barrier_abs = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_barrier_abs_m', 1.68);
            emergency_abs = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_emergency_abs_m', 1.86);
            guard_time = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_guard_time_s', 0.85);
            energy_time = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_energy_time_s', 3.20);
            cancel_time = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_cancel_time_s', 2.55);
            handoff_time = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_handoff_time_s', 4.30);

            K = RecoveryIo.loadPreferredGain(params, cfg, target_name);
            e = x - target_state;
            e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
            e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));
            u_lqr = -(K * e);

            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs([u_min, u_max]));
            xcart = x(1); vcart = x(2);
            sx = sign(xcart); if sx == 0, sx = 1; end
            outward = sx * vcart > 0;

            barrier_kp = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_barrier_kp', 32.0);
            barrier_kd = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_barrier_kd', 13.0);
            u_barrier = 0.0;
            if abs(xcart) > barrier_abs
                u_barrier = u_barrier - sx * barrier_kp * (abs(xcart) - barrier_abs) - barrier_kd * vcart;
            end
            if abs(xcart) > emergency_abs || (abs(xcart) > barrier_abs && outward)
                u_barrier = u_barrier - sx * (0.70 * umax + 9.0 * max(0, abs(xcart)-emergency_abs) + 2.8 * abs(vcart));
            end
            if abs(xcart) >= rail - 0.020 && outward
                u_barrier = -sx * 0.995 * umax;
            end
            u_barrier = min(max(u_barrier, u_min), u_max);

            timing_gain = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_timing_gain', 1.35);
            energy_gain = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_energy_gain', 0.92);
            cancel_gain = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_rate_cancel_gain', 1.55);
            lqr_min = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_terminal_lqr_min', 0.30);
            lqr_max = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_terminal_lqr_max', 0.98);
            cart_center_gain = 2.3;
            cart_damp_gain = 3.6;
            v = char(variant);
            if startsWith(v, 'upup_force_l2_ratio100')
                timing_gain = 2.25; energy_gain = 1.38; cancel_gain = 2.35;
                guard_time = max(0.35, guard_time - 0.25); energy_time = energy_time + 1.20; cancel_time = cancel_time + 0.95; handoff_time = handoff_time + 1.25;
                cart_center_gain = 1.75; cart_damp_gain = 3.05; lqr_min = 0.18; lqr_max = 0.88;
                if contains(v, 'railtiming'), cart_center_gain = 2.65; cart_damp_gain = 4.25; timing_gain = 1.95; end
                if contains(v, 'ratekill'), cancel_gain = 3.05; energy_time = energy_time - 0.25; end
            elseif startsWith(v, 'upup_force_l2_ratio075')
                timing_gain = 2.05; energy_gain = 1.28; cancel_gain = 2.15;
                guard_time = max(0.40, guard_time - 0.20); energy_time = energy_time + 0.95; cancel_time = cancel_time + 0.80; handoff_time = handoff_time + 1.05;
                cart_center_gain = 1.95; cart_damp_gain = 3.35; lqr_min = 0.20; lqr_max = 0.90;
                if contains(v, 'railtiming'), cart_center_gain = 2.85; cart_damp_gain = 4.45; timing_gain = 1.78; end
                if contains(v, 'ratekill'), cancel_gain = 2.85; energy_time = energy_time - 0.20; end
            elseif startsWith(v, 'upup_force_l1_ratio100')
                timing_gain = 1.92; energy_gain = 1.18; cancel_gain = 2.30;
                guard_time = max(0.30, guard_time - 0.35); energy_time = energy_time + 1.25; cancel_time = cancel_time + 1.05; handoff_time = handoff_time + 1.60;
                cart_center_gain = 1.65; cart_damp_gain = 2.95; lqr_min = 0.16; lqr_max = 0.86;
                if contains(v, 'ratekill'), cancel_gain = 3.15; end
                if contains(v, 'latehandoff'), lqr_min = 0.10; handoff_time = handoff_time + 1.00; end
            elseif startsWith(v, 'upup_force_after_switch')
                timing_gain = 2.10; energy_gain = 1.30; cancel_gain = 2.65;
                guard_time = max(0.45, guard_time - 0.10); energy_time = energy_time + 1.40; cancel_time = cancel_time + 1.10; handoff_time = handoff_time + 1.60;
                cart_center_gain = 1.90; cart_damp_gain = 3.50; lqr_min = 0.14; lqr_max = 0.86;
            elseif strcmp(v, 'upup_timing_l2_ratio100')
                timing_gain = 1.58; energy_gain = 1.08; cancel_gain = 1.72;
                energy_time = energy_time + 0.55; cancel_time = cancel_time + 0.25;
                cart_center_gain = 2.0; cart_damp_gain = 3.3;
            elseif strcmp(v, 'upup_timing_l2_ratio075')
                timing_gain = 1.42; energy_gain = 0.98; cancel_gain = 1.62;
                energy_time = energy_time + 0.30; cancel_time = cancel_time + 0.15;
                cart_center_gain = 2.2; cart_damp_gain = 3.5;
            elseif strcmp(v, 'upup_timing_l1_ratio100')
                timing_gain = 1.30; energy_gain = 0.90; cancel_gain = 1.50;
                handoff_time = handoff_time + 0.55;
                cart_center_gain = 2.4; cart_damp_gain = 3.7;
            elseif strcmp(v, 'upup_timing_after_switch')
                timing_gain = 1.50; energy_gain = 1.05; cancel_gain = 1.70;
                guard_time = guard_time + 0.15; energy_time = energy_time + 0.65; handoff_time = handoff_time + 0.45;
                cart_center_gain = 2.1; cart_damp_gain = 3.6;
            elseif strcmp(v, 'upup_timing_rail_guard')
                timing_gain = 0.88; energy_gain = 0.62; cancel_gain = 1.25;
                guard_time = guard_time + 0.50;
                cart_center_gain = 4.8; cart_damp_gain = 6.4;
            elseif strcmp(v, 'upup_timing_energy_sweep')
                timing_gain = 1.72; energy_gain = 1.18; cancel_gain = 1.52;
                energy_time = energy_time + 0.85;
                cart_center_gain = 1.8; cart_damp_gain = 3.0;
            end

            th_err = [e(3); e(5)];
            th_rate = [e(4); e(6)];
            angle_norm = norm(th_err);
            rate_norm = norm(th_rate);
            timing_measure = th_err(1)*th_rate(1) + 0.85*th_err(2)*th_rate(2);
            timing_dir = -sign(timing_measure + 0.18*(th_err(1)+0.75*th_err(2)));
            if timing_dir == 0
                timing_dir = -sign(u_lqr);
                if timing_dir == 0, timing_dir = 1; end
            end
            energy_blend = min(1.0, (angle_norm + 0.14*rate_norm) / 2.20);
            u_timing = timing_dir * energy_gain * energy_blend * umax;
            u_angle = timing_gain * energy_blend * u_lqr;
            u_cancel = -cancel_gain * (0.55*e(2) + e(4) + 0.78*e(6));
            u_cart = -cart_center_gain * xcart - cart_damp_gain * vcart;

            if startsWith(v, 'upup_force_')
                rail_risk = abs(xcart) > barrier_abs || (abs(xcart) > 1.35 && abs(vcart) > 0.45 && outward);
            else
                rail_risk = abs(xcart) > barrier_abs || (abs(vcart) > 0.50 && outward);
            end
            if t_rel <= guard_time || rail_risk
                stage = 'rail_guard';
                assist = 0.12*u_angle + 0.10*u_timing;
                if abs(u_barrier) > 0.10*umax && sign(assist) ~= sign(u_barrier)
                    assist = 0.04*assist;
                end
                u_raw = u_barrier + assist + 0.08*u_lqr;
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_guard_u_fraction', 0.98);
            elseif t_rel <= guard_time + energy_time
                stage = 'timing_aligned_energy_catch';
                tau = (t_rel - guard_time) / max(energy_time, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                if startsWith(v, 'upup_force_')
                    u_raw = 0.22*u_barrier + (0.42+0.36*s)*u_timing + (0.52+0.36*s)*u_angle + 0.14*u_cart;
                else
                    u_raw = 0.38*u_barrier + (0.30+0.30*s)*u_timing + (0.38+0.32*s)*u_angle + 0.20*u_cart;
                end
                if abs(xcart) > 0.92*barrier_abs && outward && sign(u_raw) == sx
                    u_raw = 0.75*u_barrier + 0.18*u_angle;
                    stage = 'timing_aligned_energy_catch_barrier_limited';
                end
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_soft_u_fraction', 0.96);
            elseif t_rel <= guard_time + energy_time + cancel_time
                stage = 'angular_velocity_cancellation';
                tau = (t_rel - guard_time - energy_time) / max(cancel_time, eps);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*lqr_min + s*min(0.82, lqr_max);
                if startsWith(v, 'upup_force_')
                    u_raw = 0.22*u_barrier + 0.72*lqr_scale*u_lqr + 0.92*u_cancel + 0.24*u_cart;
                else
                    u_raw = 0.35*u_barrier + lqr_scale*u_lqr + 0.55*u_cancel + 0.32*u_cart;
                end
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_soft_u_fraction', 0.96);
            else
                stage = 'terminal_lqr_handoff';
                tau = min(max((t_rel - guard_time - energy_time - cancel_time) / max(handoff_time, eps), 0.0), 1.0);
                s = 10*tau^3 - 15*tau^4 + 6*tau^5;
                lqr_scale = (1-s)*min(0.82, lqr_max) + s*lqr_max;
                near_balanced = angle_norm <= RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10) && ...
                                norm([e(2), e(4), e(6)]) <= RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90);
                if near_balanced && abs(xcart) > barrier_abs
                    u_raw = 1.00*u_barrier + 0.35*u_cart + 0.20*lqr_scale*u_lqr;
                    stage = 'terminal_rail_barrier_near_balanced';
                else
                    if startsWith(v, 'upup_force_')
                        u_raw = 0.20*u_barrier + lqr_scale*u_lqr + 0.20*u_cart + 0.46*u_cancel;
                    else
                        u_raw = 0.25*u_barrier + lqr_scale*u_lqr + 0.26*u_cart + 0.30*u_cancel;
                    end
                end
                soft_frac = RecoveryUtils.getDouble(cfg, 'recovery_upup_catch_soft_u_fraction', 0.96);
            end

            if abs(xcart) > emergency_abs && outward && sign(u_raw) == sx
                u_raw = min(max(u_barrier, u_min), u_max);
                stage = [stage, '_hard_barrier_override'];
            end

            u_raw = RecoveryUtils.softBoundControl(u_raw, params, cfg, soft_frac);
            u_cmd = min(max(double(u_raw), u_min), u_max);
            ctrl = struct();
            ctrl.u_cmd = u_cmd;
            ctrl.u_raw = double(u_raw);
            ctrl.recovery_state = ['upup_timing_aligned_catch_', stage, '_', char(variant)];
            ctrl.mode = ctrl.recovery_state;
            ctrl.anchor_state = anchor_state;
            ctrl.target_state = target_state;
            ctrl.target_mode = target_name;
            ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
        end

        function tf = variantIsRoaTvlqr(variant)
            v = char(variant);
            tf = startsWith(v, 'roa_traj_tvlqr');
        end

        function tf = variantIsTakeoverGuard(variant)
            v = char(variant);
            tf = startsWith(v, 'constrainedRecoverya2_') || startsWith(v, 'constrainedRecoveryb_') || startsWith(v, 'railBrakeFallback_') || startsWith(v, 'balanceFallback_') || startsWith(v, 'hardStateFallback_hs') || startsWith(v, 'targeted_railclamp_') || startsWith(v, 'ss_hs') || startsWith(v, 'lib_') || startsWith(v, 'upup_timing_') || startsWith(v, 'upup_force_') || startsWith(v, 'hard_catch_') || startsWith(v, 'terminal_') || strcmp(v, 'emergency_rail_safe_takeover') || strcmp(v, 'traj_cart_safe_takeover') || ...
                 strcmp(v, 'fail_upup_l2_high_contact') || strcmp(v, 'fail_downup_l2_high_contact') || ...
                 strcmp(v, 'fail_upup_l1_ratio1');
        end

    end
end
