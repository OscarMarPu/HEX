package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.*;

/**
 * Ejemplo de jugador Minimax que utiliza enumeración optimizada de caminos.
 * 
 * Player1 conecta izquierda->derecha
 * Player2 conecta arriba->abajo
 */
public class PlayerMinimax implements IPlayer, IAuto {
    private int depth;
    private String name;
    private PlayerType myColor;
    private long nodesVisited;
    private long nodesPruned;
    private static final int MAXIM = 1000000;

    private static final int[][] HEX_DIRECTIONS = {
        {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}
    };

    private boolean debug = true;

    public PlayerMinimax(String name, int depth) {
        this.name = name;
        this.depth = depth;
    }

    @Override
    public PlayerMove move(HexGameStatus s) {
        nodesVisited = 0;
        nodesPruned = 0;
        myColor = s.getCurrentPlayer();
        if (debug) {
            System.out.println("Color actual del jugador: " + myColor);
            printBoard(s);
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (debug) {
            System.out.println("Número de movimientos legales: " + possibleMoves.size());
        }

        if (possibleMoves.isEmpty()) {
            // No hay movimientos
            return new PlayerMove(null, nodesVisited, depth, SearchType.MINIMAX);
        }

        Point bestMove = null;
        int bestValue = Integer.MIN_VALUE;
        int alpha = Integer.MIN_VALUE;
        int beta = MAXIM;

        // Minimax principal con poda alfa-beta
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int value = minValue(newState, getOpponentColor(myColor), alpha, beta, depth - 1);
            if (value > bestValue) {
                bestValue = value;
                bestMove = move;
            }
            // Si encuentra movimiento ganador máximo, romper
            if (bestValue == MAXIM) {
                break;
            }

            alpha = Math.max(alpha, bestValue);
            if (beta <= alpha) {
                nodesPruned++;
                if (debug) {
                    System.out.println("Poda alfa-beta en: (" + move.x + ", " + move.y + ")");
                }
                break;
            }
        }

        // Fallback si no encuentra nada (poco probable)
        if (bestMove == null && !possibleMoves.isEmpty()) {
            bestMove = possibleMoves.get(0);
        }

        if (debug) {
            System.out.println("Mejor movimiento: (" + 
                (bestMove != null ? bestMove.x : "null") + ", " +
                (bestMove != null ? bestMove.y : "null") + "), Valor: " + bestValue);
        }

        // Imprimir heurísticos post-elección (no afectan la elección, sólo informativo)
        if (debug) {
            System.out.println("Heurísticos de colocar ficha en cada casilla vacía tras elegir el tiro:");
            List<Point> allMoves = getLegalMoves(s);
            for (Point m : allMoves) {
                HexGameStatus testState = applyMove(s, m);
                int hValue = evaluate(testState, myColor);
                System.out.println("Colocar en (" + m.x + ", " + m.y + ") -> Heurístico: " + hValue);
            }
        }

        return new PlayerMove(bestMove, nodesVisited, depth, SearchType.MINIMAX);
    }

    @Override
    public String getName() {
        return "Minimax(" + name + ")";
    }

    @Override
    public void timeout() {
        if (debug) {
            System.out.println("Timeout occurred.");
        }
    }

    private int maxValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int depth) {
        nodesVisited++;
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, myColor);
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            return evaluate(s, myColor);
        }

        int value = Integer.MIN_VALUE;
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int childValue = minValue(newState, getOpponentColor(currentPlayer), alpha, beta, depth - 1);
            value = Math.max(value, childValue);
            if (value >= beta) {
                nodesPruned++;
                return value;
            }
            alpha = Math.max(alpha, value);
        }
        return value;
    }

    private int minValue(HexGameStatus s, PlayerType currentPlayer, int alpha, int beta, int depth) {
        nodesVisited++;
        if (depth == 0 || s.isGameOver()) {
            return evaluate(s, myColor);
        }

        List<Point> possibleMoves = getLegalMoves(s);
        if (possibleMoves.isEmpty()) {
            return evaluate(s, myColor);
        }

        int value = Integer.MAX_VALUE;
        for (Point move : possibleMoves) {
            HexGameStatus newState = applyMove(s, move);
            int childValue = maxValue(newState, getOpponentColor(currentPlayer), alpha, beta, depth - 1);
            value = Math.min(value, childValue);
            if (value <= alpha) {
                nodesPruned++;
                return value;
            }
            beta = Math.min(beta, value);
        }
        return value;
    }

    /**
     * Evalúa el estado enumerando caminos optimizados.
     * Heurística: (SumaPuntajesCaminosPropio - SumaPuntajesCaminosOponente)*100
     * Además, si s.isGameOver() retorna MAXIM o -MAXIM según el ganador.
     */
    private int evaluate(HexGameStatus s, PlayerType evaluatorColor) {
        if (s.isGameOver()) {
            PlayerType winner = s.GetWinner();
            return (winner == myColor) ? MAXIM : -MAXIM;
        }

        double evaluatorScore = calculateAllPathsScoreOptimized(s, evaluatorColor);
        PlayerType opponentColor = getOpponentColor(evaluatorColor);
        double opponentScore = calculateAllPathsScoreOptimized(s, opponentColor);

        double heuristic = (evaluatorScore - opponentScore)*100;

        return (int) heuristic;
    }

    /**
     * Calcula el puntaje total de todos los caminos de un jugador usando la versión optimizada
     * de enumeración de caminos.
     */
    private double calculateAllPathsScoreOptimized(HexGameStatus s, PlayerType color) {
        List<List<Point>> paths = findAllPathsOptimized(s, color);
        double score = 0.0;
        for (List<Point> path : paths) {
            score += calculatePathScore(path, s.getSize());
        }
        return score;
    }

    /**
     * Encuentra todos los caminos de forma optimizada.
     */
    private List<List<Point>> findAllPathsOptimized(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int[] nodes = getVirtualNodes(color, size);
        int startNode = nodes[0];
        int endNode = nodes[1];

        // Construir grafo
        List<List<Integer>> graph = buildGraphForPaths(s, color);

        // Calcular distMin con dijkstra para limitDist
        int distMin = calculateShortestDistanceDijkstra(graph, startNode, endNode);
        if (distMin == Integer.MAX_VALUE) {
            return new ArrayList<>();
        }
        int limitDist = distMin + 2; // Limitar caminos a distMin+2
        
        // Filtrar grafo
        filterGraph(graph, startNode, endNode);

        // Enumerar caminos limitados
        List<List<Integer>> allPaths = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();
        List<Integer> currentPath = new ArrayList<>();

        findAllPathsRecursiveOptimized(graph, startNode, endNode, visited, currentPath, allPaths, limitDist, size);

        return convertPathsToPoints(allPaths, size);
    }

    private void findAllPathsRecursiveOptimized(List<List<Integer>> graph, int current, int endNode,
                                                Set<Integer> visited, List<Integer> currentPath,
                                                List<List<Integer>> allPaths, int limitDist, int size) {
        final int MAX_PATHS = 10000; // Evitar explotar
        if (allPaths.size() >= MAX_PATHS) return;

        if (current == endNode) {
            List<Integer> path = new ArrayList<>(currentPath);
            path.add(current);
            allPaths.add(path);
            return;
        }

        if (currentPath.size() >= limitDist) return;

        visited.add(current);
        currentPath.add(current);

        for (int neigh : graph.get(current)) {
            if (!visited.contains(neigh)) {
                findAllPathsRecursiveOptimized(graph, neigh, endNode, visited, currentPath, allPaths, limitDist, size);
                if (allPaths.size() >= MAX_PATHS) break;
            }
        }

        visited.remove(current);
        currentPath.remove(currentPath.size()-1);
    }

    // Funciones auxiliares:

    private double calculatePathScore(List<Point> path, int size) {
        if (path.isEmpty()) return 0.0;
        double score = 0.0;
        for (Point p : path) {
            score += centralityFactor(p, size);
        }
        return score / path.size();
    }

    private double centralityFactor(Point p, int size) {
        double centerX = (size-1)/2.0;
        double centerY = (size-1)/2.0;
        double dist = Math.sqrt(Math.pow(p.x - centerX,2)+Math.pow(p.y - centerY,2));
        return 1.0/(dist+1);
    }

    private PlayerType getOpponentColor(PlayerType color) {
        return (color == PlayerType.PLAYER1)? PlayerType.PLAYER2 : PlayerType.PLAYER1;
    }

    private int colorToInt(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? 1 : -1;
    }

    private List<Point> getLegalMoves(HexGameStatus s) {
        List<Point> moves = new ArrayList<>();
        int size = s.getSize();
        for (int i=0; i<size; i++) {
            for (int j=0; j<size; j++) {
                if (s.getPos(i,j) == 0) moves.add(new Point(i,j));
            }
        }
        return moves;
    }

    private HexGameStatus applyMove(HexGameStatus s, Point move) {
        HexGameStatus newState = new HexGameStatus(s);
        newState.placeStone(move);
        return newState;
    }

    private int[] getVirtualNodes(PlayerType color, int size) {
        int P1_START = size*size;
        int P1_END = size*size+1;
        int P2_START = size*size+2;
        int P2_END = size*size+3;
        if (color == PlayerType.PLAYER1) {
            return new int[]{P1_START, P1_END};
        } else {
            return new int[]{P2_START, P2_END};
        }
    }

    private void printBoard(HexGameStatus s) {
        int size = s.getSize();
        System.out.println("Estado actual del tablero:");
        for (int i=0;i<size;i++){
            for (int j=0;j<size;j++){
                System.out.print(s.getPos(i,j)+" ");
            }
            System.out.println();
        }
    }

    private List<Point> getStartPoints(PlayerType color, int size) {
        // Player1: izq->der
        // Player2: arriba->abajo
        List<Point> startPoints = new ArrayList<>();
        if (color == PlayerType.PLAYER1) {
            for (int y=0;y<size;y++) startPoints.add(new Point(0,y));
        } else {
            for (int x=0;x<size;x++) startPoints.add(new Point(x,0));
        }
        return startPoints;
    }

    private List<Point> getEndPoints(PlayerType color, int size) {
        List<Point> endPoints = new ArrayList<>();
        if (color == PlayerType.PLAYER1) {
            for (int y=0;y<size;y++) endPoints.add(new Point(size-1,y));
        } else {
            for (int x=0;x<size;x++) endPoints.add(new Point(x,size-1));
        }
        return endPoints;
    }

    // Construye grafo para paths
    private List<List<Integer>> buildGraphForPaths(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int totalNodes = size*size+4;
        List<List<Integer>> graph = new ArrayList<>(totalNodes);
        for (int i=0;i<totalNodes;i++) graph.add(new ArrayList<>());

        PlayerType opponent = getOpponentColor(color);
        int opponentVal = colorToInt(opponent);

        int P1_START = size*size;
        int P1_END = size*size+1;
        int P2_START = size*size+2;
        int P2_END = size*size+3;

        // Celdas transitables
        for (int x=0;x<size;x++){
            for(int y=0;y<size;y++){
                int cellVal = s.getPos(x,y);
                if (cellVal!=opponentVal) {
                    int u = x*size+y;
                    for (int[] d: HEX_DIRECTIONS) {
                        int nx=x+d[0], ny=y+d[1];
                        if (nx>=0 && nx<size && ny>=0 && ny<size) {
                            int neighVal=s.getPos(nx,ny);
                            if (neighVal!=opponentVal) {
                                int v=nx*size+ny;
                                graph.get(u).add(v);
                            }
                        }
                    }
                }
            }
        }

        // Player1: izq->der
        if (color==PlayerType.PLAYER1) {
            for (int yy=0;yy<size;yy++){
                if(s.getPos(0,yy)!=opponentVal)
                    graph.get(P1_START).add(0*size+yy);
            }
            for (int yy=0;yy<size;yy++){
                if(s.getPos(size-1,yy)!=opponentVal)
                    graph.get((size-1)*size+yy).add(P1_END);
            }
        } else {
            // Player2: arriba->abajo
            for (int xx=0;xx<size;xx++){
                if(s.getPos(xx,0)!=opponentVal)
                    graph.get(P2_START).add(xx*size+0);
            }
            for (int xx=0;xx<size;xx++){
                if(s.getPos(xx,size-1)!=opponentVal)
                    graph.get(xx*size+(size-1)).add(P2_END);
            }
        }

        return graph;
    }

    private int calculateShortestDistanceDijkstra(List<List<Integer>> graph, int startNode, int endNode) {
        int n=graph.size();
        int[] dist=new int[n];
        Arrays.fill(dist,Integer.MAX_VALUE);
        dist[startNode]=0;
        PriorityQueue<int[]>pq=new PriorityQueue<>(Comparator.comparingInt(a->a[1]));
        pq.add(new int[]{startNode,0});
        while(!pq.isEmpty()){
            int[]cur=pq.poll();
            int u=cur[0],d=cur[1];
            if (d>dist[u]) continue;
            if (u==endNode)return d;
            for(int w:graph.get(u)){
                int nd=d+1;
                if(nd<dist[w]){
                    dist[w]=nd;
                    pq.add(new int[]{w,nd});
                }
            }
        }
        return Integer.MAX_VALUE;
    }

    private void filterGraph(List<List<Integer>> graph, int startNode, int endNode) {
        Set<Integer> fromStart=bfsReachable(graph,startNode);
        List<List<Integer>> rev=buildReverseGraph(graph);
        Set<Integer> fromEnd=bfsReachable(rev,endNode);
        Set<Integer>intersection=new HashSet<>(fromStart);
        intersection.retainAll(fromEnd);

        for(int u=0;u<graph.size();u++){
            if(!intersection.contains(u)){
                graph.get(u).clear();
            } else {
                graph.get(u).removeIf(v->!intersection.contains(v));
            }
        }
    }

    private Set<Integer> bfsReachable(List<List<Integer>> g,int start){
        Set<Integer>visited=new HashSet<>();
        Queue<Integer>q=new LinkedList<>();
        visited.add(start);
        q.add(start);
        while(!q.isEmpty()){
            int u=q.poll();
            for(int w:g.get(u)){
                if(!visited.contains(w)){
                    visited.add(w);
                    q.add(w);
                }
            }
        }
        return visited;
    }

    private List<List<Integer>> buildReverseGraph(List<List<Integer>> graph) {
        int n=graph.size();
        List<List<Integer>> rev=new ArrayList<>(n);
        for(int i=0;i<n;i++) rev.add(new ArrayList<>());
        for(int u=0;u<n;u++){
            for(int v:graph.get(u)){
                rev.get(v).add(u);
            }
        }
        return rev;
    }

    private List<List<Point>> convertPathsToPoints(List<List<Integer>> paths, int size) {
        List<List<Point>> result=new ArrayList<>();
        for(List<Integer> path:paths){
            List<Point> pPath=new ArrayList<>();
            for(int n:path){
                if(n<size*size){
                    int x=n/size;
                    int y=n%size;
                    pPath.add(new Point(x,y));
                }
            }
            if(!pPath.isEmpty())result.add(pPath);
        }
        return result;
    }
}
