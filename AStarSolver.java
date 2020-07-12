package astar;

import edu.princeton.cs.algs4.Stopwatch;
import pq.ExtrinsicMinPQ;
import pq.TreeMapMinPQ;

import java.util.*;

/**
 * @see ShortestPathsSolver for more method documentation
 */
public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex> {
    private double timeSpent; //for explorationTime()
    private int stateExplored; //for numStateExplore()
    private double solutionWeight; //for solutionWeight()
    private ArrayList<Vertex> solution = new ArrayList<>(); //for solution()
    private SolverOutcome outcome;
    private Map<Vertex, Vertex> edgeTo = new HashMap<>(); //key: current vertex, value :origin
    private Map<Vertex, Double> distTo = new HashMap<>(); //key: current vertex, value: total distance from start vertex
    private boolean endIsMet = false;
    private boolean endEqualsStart = false;
    private Vertex end;
    private Vertex start;
    private AStarGraph<Vertex> graph;

    /**
     * Immediately solves and stores the result of running memory optimized A*
     * search, computing everything necessary for all other methods to return
     * their results in constant time. The timeout is given in seconds.
     */
    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout) {
        this.start = start;
        this.end = end;
        this.graph = input;
        Stopwatch sw = new Stopwatch();
        endEqualsStart = start.equals(end);
        if (!start.equals(end)) {
            ExtrinsicMinPQ<Vertex> fringe = new TreeMapMinPQ<Vertex>();
            distTo.put(start, 0.0);
            edgeTo.put(start, null);

            List<WeightedEdge<Vertex>> neighborEdges = input.neighbors(start);
            addToFringe(neighborEdges, fringe);

            while (!fringe.isEmpty()) {
                Vertex next = fringe.removeSmallest();
                if (next.equals(end)) {
                    endIsMet = true;
                    break;
                }
                stateExplored++;
                neighborEdges = input.neighbors(next);
                addToFringe(neighborEdges, fringe);
                if (timeCheck(sw, timeout)) {
                    return;
                }
            }
            if (endIsMet) {
                solutionWeight = distTo.get(end);
                shortestPath();
                return;
            } else { //end is not met
                outcome = SolverOutcome.UNSOLVABLE;
                solutionWeight = Double.POSITIVE_INFINITY;
                return;
            }
        } else { //if (endEqualsStart) {
            outcome = SolverOutcome.SOLVED;
            solutionWeight = 0.0;
            solution.add(end);
        }
    }

    private void addToFringe(List<WeightedEdge<Vertex>> neighborEdges, ExtrinsicMinPQ<Vertex> fringe) {
        Vertex startingVertex = null;
        Vertex endVertex = null;
        for (WeightedEdge<Vertex> wev : neighborEdges) {
            Vertex s = wev.from(); //getting the starting vertex
            Vertex d = wev.to(); // getting the destination vertex
            double distance = wev.weight() + distTo.get(s);

            if (!distTo.containsKey(d)) {
                distTo.put(d, distance);
                edgeTo.put(d, s);

                double priority = priority(wev.to(), end);
                fringe.add(d, priority);
            } else if (distTo.get(d) > distance) { //relaxation
                distTo.replace(d, distance);
                edgeTo.replace(d, s);

                if (fringe.contains(d)) {
                    fringe.changePriority(d, priority(d, end));
                } else {
                    fringe.add(d, priority(d, end));
                }
            }
        }
    }

    private double priority(Vertex v, Vertex goal) {
        /** calculate the priority of a given vertex v. v is element of the input.
         *  distTo(v) + h(v,goal) */
        double distance = distTo.get(v);
        double heuristic = graph.estimatedDistanceToGoal(v, goal);
        return distance + heuristic;
    }

    @Override
    public SolverOutcome outcome() {
        /** Returns one of SOLVED, TIMEOUT, or UNSOLVABLE. */
        if (endIsMet || endEqualsStart) {
            outcome = SolverOutcome.SOLVED;
        } else {
            outcome = SolverOutcome.UNSOLVABLE;
        }
        return outcome;
    }

    @Override
    public List<Vertex> solution() {
        /**
         * A list of vertices corresponding to the solution, from start to end.
         * Returns an empty list if problem was unsolvable or solving timed out.
         */
        return solution;
    }

    private void shortestPath() {
        Vertex x = end;
        solution.add(end);
        while (!x.equals(start)) {
            x = edgeTo.get(x);
            solution.add(x);
        }
        Collections.reverse(solution);
    }

    @Override
    public double solutionWeight() {
        /**
         * The total weight of the solution, taking into account edge weights.
         * Returns Double.POSITIVE_INFINITY if problem was unsolvable or solving timed out.
         */
        if (outcome == SolverOutcome.UNSOLVABLE || outcome == SolverOutcome.TIMEOUT) {
            return Double.POSITIVE_INFINITY;
        }
        return solutionWeight;
    }

    /** The total number of priority queue removeSmallest operations. */
    @Override
    public int numStatesExplored() {
        /** The total number of states explored while solving. */
        return stateExplored;
    }

    @Override
    public double explorationTime() {
        /** The total time spent in seconds by the constructor to run A* search. */
        return timeSpent;
    }

    private boolean timeCheck(Stopwatch sw, double timeout) {
        timeSpent = sw.elapsedTime();
        if (timeout < timeSpent) {
            outcome = SolverOutcome.TIMEOUT;
            return true; //timeout
        }
        return false;
    }
}
