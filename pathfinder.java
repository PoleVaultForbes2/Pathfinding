import java.lang.IndexOutOfBoundsException;
import java.lang.IllegalArgumentException;
import java.util.*;

/**
 * Pathfinder uses A* search to find a near optimal path
 * between to locations with given terrain.
 */

public class Pathfinder {
    // Reduce state map to one object that holds 3 values
    private enum NodeState {UNVISITED, OPEN, CLOSED}
    private NodeState[][] stateMap;
    //hold the directions to check the neighbors
    private static final int[][] DIRECTIONS = {{-1,0}, {1,0}, {0, -1}, {0, 1}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};
    private Coord startingCord;
    private Coord endCord;
    private final Terrain terrain;
    private PFNode endNode; // will be used to save the node that reaches the end
    
    private MinPQ<PFNode> minQ;
    private float heuristic;
    private float pathCost;
    private int searchSize;

    /**
     * PFNode will be the key for MinPQ (used in computePath())
     */
    private class PFNode implements Comparable<PFNode> {
        private Coord loc;
        private PFNode fromNode;
        // add distance to node to check it less throughout the process
        private float distanceFromStart;
        // loc: the location of the PFNode
        // fromNode: how did we get here? (linked list back to start)
        public PFNode(Coord loc, PFNode fromNode) {
            this.loc = loc;
            this.fromNode = fromNode;
            distanceFromStart = 0;
        }

        // compares this with that, used to find minimum cost PFNode
        public int compareTo(PFNode that) {
            float thisCost = this.getCost(getHeuristic());
            float thatCost = that.getCost(getHeuristic());
            // return a negative if this comes before that, positive if that comes before this
            return Float.compare(thisCost, thatCost);
        }

        // returns the cost to travel from starting point to this
        // via the fromNode chain
        public float getCost(float heuristic) {
            float heuristicCost = heuristic * terrain.computeDistance(loc.getI(), loc.getJ(), endCord.getI(), endCord.getJ());

            return distanceFromStart + heuristicCost;
        }

        // returns an Iterable of PFNodes that surround this
        public Iterable<PFNode> neighbors() {
            List<PFNode> s = new ArrayList<>(8); // max 8 neighbors
            int maxIndex = terrain.getN() -1;

            //goes through each direction and makes sure it's inbounds before adding it
            for(int[] dir : DIRECTIONS){
                int newRow = loc.getI() + dir[0];
                int newCol = loc.getJ() + dir[1];
                if(newRow >= 0 && newRow <= maxIndex && newCol >= 0 && newCol <= maxIndex){
                    s.add(new PFNode(new Coord(newRow, newCol), this));
                }
            }

            return s;
        }
        
    }

    public Pathfinder(Terrain terrain) {
        this.terrain = terrain;

        pathCost = 0;
        int size = terrain.getN();
        minQ = new MinPQ<>();
        endNode = null;
        startingCord = null;
        endCord = null;
        stateMap = new NodeState[size][size];

        for(int i = 0; i < size; i++){
            for(int j=0; j < size; j++){
                stateMap[i][j] = NodeState.UNVISITED;
            }
        }

    }

    public void setPathStart(Coord loc) {
        if(loc == null) throw new IllegalArgumentException("Can't pass in null");
        else startingCord = loc;
    }

    public Coord getPathStart() {
        return startingCord;
    }

    public void setPathEnd(Coord loc) {
        if(loc == null) throw new IllegalArgumentException("Can't pass in null");
        else endCord = loc;
    }

    public Coord getPathEnd() {
        return endCord;
    }

    public PFNode getEndNode(){return endNode;}

    public void setEndNode(PFNode end){endNode = end;}

    public void setHeuristic(float v) { heuristic = v;
    }

    public float getHeuristic() {
        return heuristic;
    }

    public void resetPath() {
        int size = terrain.getN();

        for(int i = 0; i < size; i++){
            for(int j=0; j < size; j++){
                stateMap[i][j] = NodeState.UNVISITED;
            }
        }

        minQ = new MinPQ<>();
    }

    public void computePath() {
        if(getPathStart() == null || getPathEnd() == null) throw new IllegalArgumentException("Have to set start and end before running");

        PFNode start = new PFNode(getPathStart(), null); // came from nothing
        start.distanceFromStart = 0;
        minQ.insert(start);
        stateMap[start.loc.getI()][start.loc.getJ()] = NodeState.OPEN;
        searchSize = 0;

        // find the min and check if it's the end
        while(!minQ.isEmpty()){
            // checking the minimum thing and deleting it from the queue
            PFNode current = minQ.delMin();
            searchSize++;

            if (current.loc.equals(getPathEnd())){
                //System.out.println("We found the end");
                stateMap[current.loc.getI()][current.loc.getJ()] = NodeState.CLOSED;
                setEndNode(current);
                getPathSolution();
                return;
            }

            // we have now been to this location
            stateMap[current.loc.getI()][current.loc.getJ()] = NodeState.CLOSED;

            // check neighbors
            for(PFNode neighbor : current.neighbors()){
                //if we haven't been to the neighbor continue
                if(stateMap[neighbor.loc.getI()][neighbor.loc.getJ()] == NodeState.CLOSED) continue;

                // newCost is used to find if there is a faster way to reach that node from a different neighbor
                float newCost = current.distanceFromStart + terrain.computeTravelCost(current.loc, neighbor.loc);
                // if we haven't been there and this path is more efficient then change the path to this node
                if(stateMap[neighbor.loc.getI()][neighbor.loc.getJ()] == NodeState.UNVISITED || newCost < neighbor.distanceFromStart){
                    neighbor.fromNode = current;
                    neighbor.distanceFromStart = newCost;
                    // if the new neighbor isn't already in the queue then add it 
                    if(stateMap[neighbor.loc.getI()][neighbor.loc.getJ()] != NodeState.OPEN){
                        minQ.insert(neighbor);
                        stateMap[neighbor.loc.getI()][neighbor.loc.getJ()] = NodeState.OPEN;
                    }
                }
            }
        }
    }

    public void findPath(ArrayList<Coord> path, PFNode current){
        // iterate through the linked list to find path
        while(current != null){
            path.add(current.loc);
            // get the path cost from one node to the next
            if(current.fromNode != null){
                pathCost += terrain.computeTravelCost(current.fromNode.loc.getI(), current.fromNode.loc.getJ(), current.loc.getI(), current.loc.getJ());
            }
            current = current.fromNode;
        }
    }

    public boolean foundPath() {
        return stateMap[endCord.getI()][endCord.getJ()] == NodeState.CLOSED;
    }

    public float getPathCost() {
        if(foundPath()) return pathCost;
        return Float.POSITIVE_INFINITY;
    }

    public int getSearchSize() {
        return searchSize;
    }

    public Iterable<Coord> getPathSolution() {
        ArrayList<Coord> path = new ArrayList<>();
        if(foundPath()) {
            pathCost = 0; //resetting path cost
            findPath(path, getEndNode());
            Collections.reverse(path);
        }else {
            return null;
        }

        return path;
    }

    public boolean wasSearched(Coord loc) {
        return stateMap[loc.getI()][loc.getJ()] != NodeState.UNVISITED;
    }
}
