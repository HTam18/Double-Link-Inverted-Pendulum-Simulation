function summary = runSwitchingDemo(user_cfg)
    if nargin < 1
        user_cfg = struct();
    end
    summary = runSwitchingDemoCore(user_cfg);
end
