function [out, ctx] = hybridMultitargetController(t, state, ctx, params)
    [out, ctx] = hybridMultitargetControllerCore(t, state, ctx, params);
end
