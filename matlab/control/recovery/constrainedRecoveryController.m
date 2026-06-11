function ctrl = constrainedRecoveryController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)
    ctrl = constrainedRecoveryControllerCore(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant);
end
