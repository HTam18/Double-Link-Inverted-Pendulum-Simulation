classdef RecoveryVariants
    methods (Static)
        function variants = buildBalanceFallbackVariantList(target_name, event, cfg)
            variants = {};
            is_1n = abs(double(event.force_N) - 1.0) < 1e-9;
            if ~is_1n, return; end
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            is_after = strcmp(ct, 'after_full_switching');
            ratio = double(event.contact_ratio);
            link_id = double(event.link_id);
            tgt = char(target_name);

            if RecoveryUtils.getLogical(cfg, 'recovery_balanceFallbacka_rail_only_enable', true)
                if strcmp(tgt, 'down_up') && link_id == 2 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbacka_downup_l2_r100_predictive_rail_left','balanceFallbacka_downup_l2_r100_predictive_rail_right','balanceFallbacka_railonly_predictive_soft'}];
                elseif is_after && strcmp(tgt, 'up_up') && link_id == 1 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbacka_after_upup_l1_r100_railonly','balanceFallbacka_railonly_predictive_soft'}];
                elseif is_after && strcmp(tgt, 'up_up') && link_id == 2 && ratio >= 0.75 - 1e-12
                    variants = [variants, {'balanceFallbacka_after_upup_l2_highcontact_rail_guard','balanceFallbacka_railonly_predictive_soft'}];
                end
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_balanceFallbackb_upup_directed_enable', true) && strcmp(tgt, 'up_up')
                if is_after && link_id == 2 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbackb_after_upup_l2_r100_swingcatch','balanceFallbackb_after_upup_l2_r100_ratekill','balanceFallbackb_after_upup_l2_r100_railfirst'}];
                elseif is_after && link_id == 2 && ratio >= 0.75 - 1e-12
                    variants = [variants, {'balanceFallbackb_after_upup_l2_r075_swingcatch','balanceFallbackb_after_upup_l2_r075_ratekill','balanceFallbackb_after_upup_l2_r075_railfirst'}];
                elseif is_after && link_id == 1 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbackb_after_upup_l1_r100_swingcatch','balanceFallbackb_after_upup_l1_r100_ratekill'}];
                elseif link_id == 1 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbackb_upup_l1_r100_swingcatch','balanceFallbackb_upup_l1_r100_ratekill','balanceFallbackb_upup_l1_r100_longcatch'}];
                elseif link_id == 2 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'balanceFallbackb_upup_l2_r100_swingcatch','balanceFallbackb_upup_l2_r100_ratekill','balanceFallbackb_upup_l2_r100_railfirst'}];
                elseif link_id == 2 && ratio >= 0.75 - 1e-12
                    variants = [variants, {'balanceFallbackb_upup_l2_r075_swingcatch','balanceFallbackb_upup_l2_r075_ratekill','balanceFallbackb_upup_l2_r075_railfirst'}];
                end
            end
            variants = unique(variants, 'stable');
        end

        function variants = buildConstrainedRecoveryVariantList(target_name, event, cfg)
            variants = {};
            if abs(double(event.force_N) - 1.0) > 1e-9
                return;
            end
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            is_after = strcmp(ct, 'after_full_switching');
            ratio = double(event.contact_ratio);
            link_id = double(event.link_id);
            tgt = char(target_name);
            if isfield(event, 'direction_name')
                dirn = char(event.direction_name);
            elseif isfield(event, 'direction')
                dirn = char(event.direction);
            else
                dirn = 'right';
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_constrainedRecoverya2_microbrake_enable', true)
                if is_after && strcmp(tgt, 'up_up') && link_id == 1 && ratio >= 1.0 - 1e-12
                    if strcmp(dirn, 'right')
                        variants = [variants, {'constrainedRecoverya2_after_upup_l1_r100_right_soft','constrainedRecoverya2_after_upup_l1_r100_right_mid','constrainedRecoverya2_after_upup_l1_r100_right_late'}];
                    else
                        variants = [variants, {'constrainedRecoverya2_after_upup_l1_r100_left_soft','constrainedRecoverya2_after_upup_l1_r100_left_mid'}];
                    end
                elseif ~is_after && strcmp(tgt, 'down_up') && link_id == 2 && ratio >= 1.0 - 1e-12
                    if strcmp(dirn, 'left')
                        variants = [variants, {'constrainedRecoverya2_downup_l2_r100_left_soft','constrainedRecoverya2_downup_l2_r100_left_mid','constrainedRecoverya2_downup_l2_r100_left_late'}];
                    else
                        variants = [variants, {'constrainedRecoverya2_downup_l2_r100_right_soft','constrainedRecoverya2_downup_l2_r100_right_mid'}];
                    end
                end
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_constrainedRecoveryb_upup_hardstate_enable', true) && strcmp(tgt, 'up_up')
                if is_after && link_id == 2 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_after_upup_l2_r100_rail','constrainedRecoveryb_after_upup_l2_r100_slow','constrainedRecoveryb_after_upup_l2_r100_aggressive'}];
                elseif is_after && link_id == 2 && ratio >= 0.75 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_after_upup_l2_r075_rail','constrainedRecoveryb_after_upup_l2_r075_slow','constrainedRecoveryb_after_upup_l2_r075_aggressive'}];
                elseif is_after && link_id == 1 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_after_upup_l1_r100_rail','constrainedRecoveryb_after_upup_l1_r100_slow'}];
                elseif link_id == 1 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_upup_l1_r100_rail','constrainedRecoveryb_upup_l1_r100_slow','constrainedRecoveryb_upup_l1_r100_aggressive'}];
                elseif link_id == 2 && ratio >= 1.0 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_upup_l2_r100_rail','constrainedRecoveryb_upup_l2_r100_slow','constrainedRecoveryb_upup_l2_r100_aggressive'}];
                elseif link_id == 2 && ratio >= 0.75 - 1e-12
                    variants = [variants, {'constrainedRecoveryb_upup_l2_r075_rail','constrainedRecoveryb_upup_l2_r075_slow','constrainedRecoveryb_upup_l2_r075_aggressive'}];
                end
            end
            variants = unique(variants, 'stable');
        end

        function variants = buildHardStateLibraryVariants(target_name, event, cfg)
            variants = {};
            if abs(double(event.force_N) - 1.0) > 1e-9
                return;
            end
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            is_after = strcmp(ct, 'after_full_switching');
            dir = char(event.direction_name);

            if RecoveryUtils.getLogical(cfg, 'recovery_targeted_rail_clamp_enable', false)
                ss_id_for_clamp = RecoveryVariants.stateSpecificHardStateId(ct, target_name, event);
                clamp_ids = RecoveryUtils.getCell(cfg, 'recovery_targeted_rail_clamp_hard_state_ids', {'hs09','hs14'});
                if ~isempty(ss_id_for_clamp) && any(strcmp(ss_id_for_clamp, clamp_ids))
                    variants{end+1} = ['targeted_railclamp_', ss_id_for_clamp]; %#ok<AGROW>
                end
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_hardStateFallback_constrained_hardstate_enable', false)
                hardStateFallback_id = RecoveryVariants.stateSpecificHardStateId(ct, target_name, event);
                if ~isempty(hardStateFallback_id)
                    variants{end+1} = ['hardStateFallback_', hardStateFallback_id]; %#ok<AGROW>
                    variants{end+1} = ['hardStateFallback_', hardStateFallback_id, '_soft']; %#ok<AGROW>
                    variants{end+1} = ['hardStateFallback_', hardStateFallback_id, '_rail']; %#ok<AGROW>
                end
            end

            if RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg) && RecoveryUtils.getLogical(cfg, 'recovery_state_specific_gain_search_enable', false)
                ss_id = RecoveryVariants.stateSpecificHardStateId(ct, target_name, event);
                if ~isempty(ss_id)
                    ids = RecoveryUtils.getVector(cfg, 'recovery_state_specific_gain_ids', 1:18);
                    for ii = 1:numel(ids)
                        variants{end+1} = sprintf('ss_%s_g%02d', ss_id, round(double(ids(ii)))); %#ok<AGROW>
                    end
                    variants{end+1} = sprintf('ss_%s_railclamp', ss_id); %#ok<AGROW>
                    variants{end+1} = sprintf('ss_%s_anglecatch', ss_id); %#ok<AGROW>
                    variants{end+1} = sprintf('ss_%s_latehandoff', ss_id); %#ok<AGROW>
                end
            end

            if strcmp(char(target_name), 'up_up') && double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12
                if is_after
                    variants = [variants, {['lib_after_upup_l1_r100_', dir, '_pre_angle'], ['lib_after_upup_l1_r100_', dir, '_ratekill'], ['lib_after_upup_l1_r100_', dir, '_late_lqr']}];
                else
                    variants = [variants, {['lib_target_upup_l1_r100_', dir, '_angle'], ['lib_target_upup_l1_r100_', dir, '_sweep'], ['lib_target_upup_l1_r100_', dir, '_ratekill']}];
                end
            elseif strcmp(char(target_name), 'up_up') && double(event.link_id) == 2 && abs(double(event.contact_ratio)-0.75) < 1e-9
                if is_after
                    variants = [variants, {['lib_after_upup_l2_r075_', dir, '_pre_railangle'], ['lib_after_upup_l2_r075_', dir, '_ratekill'], ['lib_after_upup_l2_r075_', dir, '_railbarrier']}];
                else
                    variants = [variants, {['lib_target_upup_l2_r075_', dir, '_railangle'], ['lib_target_upup_l2_r075_', dir, '_angle'], ['lib_target_upup_l2_r075_', dir, '_ratekill']}];
                end
            elseif strcmp(char(target_name), 'up_up') && double(event.link_id) == 2 && double(event.contact_ratio) >= 1.0 - 1e-12
                if is_after
                    variants = [variants, {['lib_after_upup_l2_r100_', dir, '_pre_railangle'], ['lib_after_upup_l2_r100_', dir, '_ratekill'], ['lib_after_upup_l2_r100_', dir, '_longcatch']}];
                else
                    variants = [variants, {['lib_target_upup_l2_r100_', dir, '_railangle'], ['lib_target_upup_l2_r100_', dir, '_angle'], ['lib_target_upup_l2_r100_', dir, '_longcatch']}];
                end
            elseif strcmp(char(target_name), 'down_up') && double(event.link_id) == 2 && double(event.contact_ratio) >= 1.0 - 1e-12 && ~is_after
                variants = [variants, {['lib_target_downup_l2_r100_', dir, '_railbarrier'], ['lib_target_downup_l2_r100_', dir, '_velocity'], ['lib_target_downup_l2_r100_', dir, '_catch']}];
            end
            variants = unique(variants, 'stable');
        end

        function variants = buildRailBrakeFallbackVariantList(target_name, event, cfg)
            variants = {};
            is_1n = abs(double(event.force_N) - 1.0) < 1e-9;
            if ~is_1n, return; end
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            is_after = strcmp(ct, 'after_full_switching');
            ratio = double(event.contact_ratio);
            link_id = double(event.link_id);
            tgt = char(target_name);
            if isfield(event, 'direction_name')
                dirn = char(event.direction_name);
            elseif isfield(event, 'direction')
                dirn = char(event.direction);
            else
                dirn = 'right';
            end

            if strcmp(tgt, 'down_up') && ~is_after && link_id == 2 && ratio >= 1.0 - 1e-12
                if strcmp(dirn, 'left')
                    variants = {'railBrakeFallback_downup_l2_r100_left_early','railBrakeFallback_downup_l2_r100_left_strong','railBrakeFallback_downup_l2_r100_left_soft'};
                else
                    variants = {'railBrakeFallback_downup_l2_r100_right_early','railBrakeFallback_downup_l2_r100_right_strong'};
                end
            elseif strcmp(tgt, 'up_up') && is_after && link_id == 1 && ratio >= 1.0 - 1e-12
                if strcmp(dirn, 'right')
                    variants = {'railBrakeFallback_after_upup_l1_r100_right_early','railBrakeFallback_after_upup_l1_r100_right_strong','railBrakeFallback_after_upup_l1_r100_right_soft'};
                else
                    variants = {'railBrakeFallback_after_upup_l1_r100_left_soft','railBrakeFallback_after_upup_l1_r100_left_early'};
                end
            end
            variants = unique(variants, 'stable');
        end

        function tf = isPriority1nFailState(target_name, event, cfg)
            ratio_gate = RecoveryUtils.getDouble(cfg, 'recovery_failstate_ratio_gate', 0.75);
            is_1n = abs(double(event.force_N) - 1.0) < 1e-9;
            high_ratio = double(event.contact_ratio) >= ratio_gate;
            tf = is_1n && high_ratio && ...
                 ((strcmp(char(target_name), 'up_up') && double(event.link_id) == 2) || ...
                  (strcmp(char(target_name), 'down_up') && double(event.link_id) == 2) || ...
                  (strcmp(char(target_name), 'up_up') && double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12));
        end

        function [hs, gid, suffix] = parseStateSpecificVariant(variant)
            v = char(variant);
            hs = ''; gid = 0; suffix = '';
            tok = regexp(v, '^ss_(hs\d\d)_g(\d+)$', 'tokens', 'once');
            if ~isempty(tok)
                hs = tok{1}; gid = str2double(tok{2}); suffix = 'grid'; return;
            end
            tok = regexp(v, '^ss_(hs\d\d)_(.*)$', 'tokens', 'once');
            if ~isempty(tok)
                hs = tok{1}; suffix = tok{2}; gid = 0; return;
            end
        end

        function [lock_case, lock_variant] = recoveryLookupBest50Lock(target_name, event, cfg)
            lock_case = false;
            lock_variant = 'local_lqr';
            if ~RecoveryUtils.getLogical(cfg, 'recovery_best50_lock_enable', false)
                return;
            end
            csv_path = RecoveryUtils.getChar(cfg, 'recovery_best50_lock_csv', '');
            if isempty(csv_path) || ~isfile(csv_path)
                return;
            end
            persistent cached_csv cached_table
            if isempty(cached_csv) || ~strcmp(cached_csv, csv_path)
                try
                    cached_table = readtable(csv_path);
                    cached_csv = csv_path;
                catch
                    cached_table = table();
                    cached_csv = csv_path;
                end
            end
            T = cached_table;
            if isempty(T) || ~all(ismember({'case_type','target','link_id','contact_ratio','direction','force_N','success','planner_selected_variant'}, T.Properties.VariableNames))
                return;
            end
            ct = RecoveryUtils.getChar(cfg, 'recovery_current_case_type', 'target_recovery');
            dir = char(event.direction_name);
            mask = strcmp(T.case_type, ct) & strcmp(T.target, char(target_name)) & ...
                   double(T.link_id) == double(event.link_id) & ...
                   abs(double(T.contact_ratio) - double(event.contact_ratio)) < 1e-9 & ...
                   strcmp(T.direction, dir) & abs(double(T.force_N) - double(event.force_N)) < 1e-9 & ...
                   logical(T.success);
            idx = find(mask, 1, 'first');
            if isempty(idx), return; end
            lock_case = true;
            lock_variant = char(T.planner_selected_variant(idx));
            if isempty(lock_variant) || strcmpi(lock_variant, 'unknown')
                lock_variant = 'local_lqr';
            end
        end

        function gp = stateSpecificGainParams(hs, gid, suffix)
            gp = struct();
            gp.pre_t = 0.0; gp.guard_t = 0.35; gp.catch_t = 5.4; gp.cancel_t = 4.2; gp.handoff_t = 5.6;
            gp.barrier_abs = 1.70; gp.emergency_abs = 1.90; gp.barrier_kp = 32.0; gp.barrier_kd = 13.0;
            gp.timing_gain = 2.5; gp.energy_gain = 1.3; gp.cancel_gain = 2.6; gp.cart_gain = 0.8; gp.cart_damp = 2.0;
            gp.lqr_min = 0.04; gp.lqr_max = 0.88; gp.soft_frac = 0.98; gp.pulse_sign = 1; gp.pulse_frac = 0.36;
            gp.angle_first = true; gp.rail_first = false; gp.near_balanced_rail_clamp = false;
            hs_num = str2double(hs(3:4));
            is_after = hs_num >= 9;
            is_link1 = any(hs_num == [1 2 9 10]);
            is_link2_075 = any(hs_num == [3 4 11 12]);
            is_link2_100 = any(hs_num == [5 6 13 14]);
            is_downup = any(hs_num == [7 8]);
            is_left = any(hs_num == [2 4 6 8 10 12 14]);
            if is_after
                gp.pre_t = 1.25; gp.catch_t = gp.catch_t + 1.6; gp.cancel_t = gp.cancel_t + 1.1; gp.handoff_t = gp.handoff_t + 1.0; gp.lqr_max = 0.84;
            end
            if is_link1
                gp.barrier_abs = 1.84; gp.emergency_abs = 1.94; gp.barrier_kp = 16; gp.barrier_kd = 7;
                gp.timing_gain = 3.3; gp.energy_gain = 1.8; gp.cancel_gain = 3.5; gp.cart_gain = 0.25; gp.cart_damp = 1.0; gp.catch_t = gp.catch_t + 1.8;
                gp.angle_first = true; gp.rail_first = false;
            elseif is_link2_075
                gp.barrier_abs = 1.56; gp.emergency_abs = 1.84; gp.barrier_kp = 44; gp.barrier_kd = 18;
                gp.timing_gain = 2.8; gp.energy_gain = 1.55; gp.cancel_gain = 3.2; gp.cart_gain = 1.2; gp.cart_damp = 3.0; gp.rail_first = true;
            elseif is_link2_100
                gp.barrier_abs = 1.50; gp.emergency_abs = 1.82; gp.barrier_kp = 54; gp.barrier_kd = 22;
                gp.timing_gain = 3.0; gp.energy_gain = 1.75; gp.cancel_gain = 3.6; gp.cart_gain = 1.45; gp.cart_damp = 3.7; gp.rail_first = true; gp.catch_t = gp.catch_t + 1.0;
            elseif is_downup
                gp.barrier_abs = 1.42; gp.emergency_abs = 1.78; gp.barrier_kp = 70; gp.barrier_kd = 28;
                gp.timing_gain = 1.5; gp.energy_gain = 0.95; gp.cancel_gain = 4.2; gp.cart_gain = 2.4; gp.cart_damp = 5.2; gp.rail_first = true; gp.near_balanced_rail_clamp = true;
            end
            if is_left, gp.pulse_sign = -1; else, gp.pulse_sign = 1; end
            if nargin >= 3 && ~isempty(suffix) && ~strcmp(suffix, 'grid')
                if strcmp(suffix, 'railclamp')
                    gp.near_balanced_rail_clamp = true; gp.rail_first = true; gp.barrier_abs = min(gp.barrier_abs, 1.38); gp.barrier_kp = gp.barrier_kp*1.45; gp.barrier_kd = gp.barrier_kd*1.35; gp.timing_gain = gp.timing_gain*0.65;
                elseif strcmp(suffix, 'anglecatch')
                    gp.angle_first = true; gp.rail_first = false; gp.timing_gain = gp.timing_gain*1.35; gp.energy_gain = gp.energy_gain*1.25; gp.cancel_gain = gp.cancel_gain*1.15; gp.barrier_abs = min(1.90, gp.barrier_abs+0.16);
                elseif strcmp(suffix, 'latehandoff')
                    gp.catch_t = gp.catch_t + 2.0; gp.cancel_t = gp.cancel_t + 1.4; gp.handoff_t = gp.handoff_t + 1.4; gp.lqr_min = 0.02; gp.lqr_max = min(0.92, gp.lqr_max+0.04);
                end
                return;
            end
            g = max(1, min(18, round(double(gid))));
            sign_set = [1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1];
            timing_mul = [0.85 0.85 1.05 1.05 1.25 1.25 1.45 1.45 1.05 1.05 1.30 1.30 0.75 0.75 1.55 1.55 1.15 1.15];
            cancel_mul = [0.90 0.90 1.00 1.00 1.10 1.10 1.25 1.25 1.55 1.55 1.75 1.75 2.05 2.05 1.30 1.30 1.90 1.90];
            rail_mul = [1.00 1.00 1.00 1.00 0.85 0.85 0.80 0.80 1.15 1.15 1.25 1.25 1.45 1.45 0.95 0.95 1.35 1.35];
            time_add = [0 0 .6 .6 1.2 1.2 1.8 1.8 .3 .3 .8 .8 1.0 1.0 2.2 2.2 1.5 1.5];
            gp.pulse_sign = gp.pulse_sign * sign_set(g);
            gp.timing_gain = gp.timing_gain * timing_mul(g);
            gp.energy_gain = gp.energy_gain * sqrt(timing_mul(g));
            gp.cancel_gain = gp.cancel_gain * cancel_mul(g);
            gp.barrier_kp = gp.barrier_kp * rail_mul(g);
            gp.barrier_kd = gp.barrier_kd * sqrt(rail_mul(g));
            gp.catch_t = gp.catch_t + time_add(g);
            gp.cancel_t = gp.cancel_t + 0.6*time_add(g);
            gp.pulse_frac = 0.24 + 0.035*g;
            if g >= 13
                gp.rail_first = true; gp.barrier_abs = min(gp.barrier_abs, 1.48);
            end
            if g == 15 || g == 16
                gp.angle_first = true; gp.rail_first = false; gp.barrier_abs = min(1.92, gp.barrier_abs + 0.18);
            end
        end

        function hs = stateSpecificHardStateId(case_type, target_name, event)
            hs = '';
            if abs(double(event.force_N) - 1.0) > 1e-9
                return;
            end
            ct = char(case_type);
            tg = char(target_name);
            dir = char(event.direction_name);
            link = double(event.link_id);
            ratio = double(event.contact_ratio);
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 1 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'right'), hs = 'hs01'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 1 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'left'),  hs = 'hs02'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-0.75) < 1e-9 && strcmp(dir,'right'), hs = 'hs03'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-0.75) < 1e-9 && strcmp(dir,'left'),  hs = 'hs04'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'right'), hs = 'hs05'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'left'),  hs = 'hs06'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'down_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'right'), hs = 'hs07'; return; end
            if strcmp(ct, 'target_recovery') && strcmp(tg, 'down_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'left'),  hs = 'hs08'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 1 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'right'), hs = 'hs09'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 1 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'left'),  hs = 'hs10'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-0.75) < 1e-9 && strcmp(dir,'right'), hs = 'hs11'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-0.75) < 1e-9 && strcmp(dir,'left'),  hs = 'hs12'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'right'), hs = 'hs13'; return; end
            if strcmp(ct, 'after_full_switching') && strcmp(tg, 'up_up') && link == 2 && abs(ratio-1.00) < 1e-9 && strcmp(dir,'left'),  hs = 'hs14'; return; end
        end

        function [T, track_scale, terminal_scale, cart_gain, cart_damp, angle_damp] = variantParameters(variant, cfg)
            switch char(variant)
                case {'roa_traj_tvlqr_fast'}
                    T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_fast_duration_s', 3.2);
                    track_scale = 0.85; terminal_scale = 0.85; cart_gain = 4.0; cart_damp = 1.8; angle_damp = 0.20;
                case {'roa_traj_tvlqr_safe'}
                    T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_safe_duration_s', 4.8);
                    track_scale = 0.70; terminal_scale = 0.75; cart_gain = 5.5; cart_damp = 2.4; angle_damp = 0.28;
                case {'roa_traj_tvlqr_slow'}
                    T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_slow_duration_s', 6.0);
                    track_scale = 0.60; terminal_scale = 0.70; cart_gain = 6.0; cart_damp = 2.6; angle_damp = 0.30;
                case {'roa_traj_tvlqr_nominal'}
                    T = RecoveryUtils.getDouble(cfg, 'recovery_roa_tvlqr_nominal_duration_s', 4.2);
                    track_scale = 0.75; terminal_scale = 0.80; cart_gain = 4.8; cart_damp = 2.0; angle_damp = 0.24;
                case 'traj_fast'
                    T = RecoveryUtils.getDouble(cfg, 'recovery_traj_fast_duration_s', 1.6);
                    track_scale = 1.05; terminal_scale = 1.05; cart_gain = 2.0; cart_damp = 0.8; angle_damp = 0.10;
                case 'traj_slow'
                    T = RecoveryUtils.getDouble(cfg, 'recovery_traj_slow_duration_s', 4.4);
                    track_scale = 0.75; terminal_scale = 0.85; cart_gain = 2.8; cart_damp = 1.0; angle_damp = 0.14;
                case 'traj_cart_safe'
                    T = RecoveryUtils.getDouble(cfg, 'recovery_traj_cart_safe_duration_s', 3.4);
                    track_scale = 0.80; terminal_scale = 0.80;
                    cart_gain = RecoveryUtils.getDouble(cfg, 'recovery_traj_cart_safe_gain', 4.5);
                    cart_damp = RecoveryUtils.getDouble(cfg, 'recovery_traj_cart_safe_damping', 1.6);
                    angle_damp = RecoveryUtils.getDouble(cfg, 'recovery_traj_angle_damping', 0.18);
                case 'traj_aggressive'
                    T = RecoveryUtils.getDouble(cfg, 'recovery_traj_aggressive_duration_s', 2.2);
                    track_scale = 1.20; terminal_scale = 1.25; cart_gain = 1.6; cart_damp = 0.55; angle_damp = 0.08;
                otherwise
                    T = RecoveryUtils.getDouble(cfg, 'recovery_traj_nominal_duration_s', 2.8);
                    track_scale = RecoveryUtils.getDouble(cfg, 'recovery_traj_track_scale', 0.80);
                    terminal_scale = RecoveryUtils.getDouble(cfg, 'recovery_traj_terminal_scale', 1.00);
                    cart_gain = 2.2; cart_damp = 0.9; angle_damp = 0.12;
            end
        end

    end
end
