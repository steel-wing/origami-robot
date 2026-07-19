import nevergrad as ng
import concurrent.futures
import multiprocessing
from runner import bez_sim

pi = 3.14159
npoints = 4
ndims = 3 * npoints

# total number of timesteps to explore
budget = 2000

bez_bounds_low =  [0 for _ in range(ndims)]
bez_bounds_high = [3 for _ in range(ndims)]

       # a, a    a, m    a, l    m, m    m, l    l, l    l, r
move = [(4, 4), (4, 3), (4, 1), (3, 3), (3, 1), (1, 1), (1, 2)]
# orientation. 1 is generally considered "up"
rotate = [1, 0]
#          y, x, rotation
penalty = [1, 0, 2]

global gleft, gright, gloss, gorient
gleft, gright = move[0]
gorient = rotate[0]
gloss = penalty[0]

def evaluate_gait(params):
    cont = params["cont"]

    x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4 = cont
    x, y, rot = bez_sim(
        x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4,
        left=gleft,
        right=gright,
        orient=gorient,
        loss=gloss,
        display=False,
        printy=False,
    )

    # maximize distance
    if gloss == 0:
        return -abs(x)
    elif gloss == 1:
        return -abs(y)
    elif gloss == 2:
        return -abs(rot)

if __name__ == "__main__":
    for gleft, gright in move:
        for gloss in penalty:
            for gorient in rotate:

                param_space = ng.p.Dict(
                    # continuous parameters
                    cont=ng.p.Array(shape=(ndims,))
                    .set_bounds(bez_bounds_low, bez_bounds_high)
                    .set_mutation(sigma=0.5),
                )

                loss_string = ["x displacement", "y displacement", "rotation"]

                print(f"\n\tTesting mode ({gleft}, {gright}), with orientation {gorient}, with {loss_string[gloss]} loss")

                optimizer = ng.optimizers.CMA(parametrization=param_space, budget=budget, num_workers=multiprocessing.cpu_count())
                
                # optional initial guess
                # optimizer.suggest({'cont': [0.2, 0.2, 2.6, 2.4, 0.9, 0.2, 1.8, 2.6, 0.2, 0.2, 0.2, 0.2]})

                with concurrent.futures.ProcessPoolExecutor() as executor:
                    recommendation = optimizer.minimize(evaluate_gait, executor=executor)
                rec = recommendation.value

                bez_sim(*rec["cont"],   # unpack the continuous params
                    left=gleft,
                    right=gright,
                    orient=gorient,
                    loss=gloss,
                    display=False,
                    printy=True
                )
