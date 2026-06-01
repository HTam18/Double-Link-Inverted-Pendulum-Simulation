% RUN_TVLQR_TRACKING_TEST Phase 29 compatibility wrapper.
%
% The clean Phase 29 implementation is in run_phase29_clean_test.m.
% This wrapper keeps the expected Phase 29 script name from the roadmap.
%
% Important:
% MATLAB run() temporarily changes the current folder to the folder of the
% script being executed.  Therefore this wrapper must call the clean test by
% using its absolute path, not by using 'matlab/simulation/...' again.

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
run(fullfile(simulation_dir, 'run_phase29_clean_test.m'));
