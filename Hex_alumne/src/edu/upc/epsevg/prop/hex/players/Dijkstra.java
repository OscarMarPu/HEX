package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.awt.Point;
import java.util.*;

public class Dijkstra {

    
    private int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        // cell == -playerId => rival
        return (cell == -playerId) ? Integer.MAX_VALUE : 1;
    }

    public int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
    int size = gameState.getSize();
    int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;

    // Crear lista de nodos fuente según el jugador.
    List<Point> fuentes = (p == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);

    // Si no hay fuentes transitables, retornar distancia muy grande.
    if (fuentes.isEmpty()) return 999999;

    // Inicializar matrices de distancia.
    int[][] dist = new int[size][size];
    for (int i = 0; i < size; i++) {
        Arrays.fill(dist[i], Integer.MAX_VALUE);
    }

    // PriorityQueue optimizada para Dijkstra.
    PriorityQueue<HexHeuristica.Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

    // Inicializar distancias desde las fuentes.
    for (Point f : fuentes) {
        int cost = getCost(gameState, f.x, f.y, playerId);
        if (cost != Integer.MAX_VALUE) {
            dist[f.x][f.y] = cost;
            pq.add(new HexHeuristica.Node(f.x, f.y, cost));
        }
    }

    // Definir movimientos posibles en Hex.
    Point[] directions = {
        new Point(0, 1), new Point(1, 0), new Point(1, -1),
        new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
    };

    // Ejecutar Dijkstra.
    while (!pq.isEmpty()) {
        HexHeuristica.Node current = pq.poll();

        // Ignorar nodos ya procesados con menor distancia.
        if (current.distance > dist[current.x][current.y]) continue;

        // Verificar si se ha llegado al borde contrario.
        if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
            (p == PlayerType.PLAYER2 && current.y == size - 1)) {
            //System.out.println("Llegó al borde en (" + current.x + ", " + current.y + ") con distancia: " + current.distance);
            return current.distance;
        }

        // Explorar vecinos.
        for (Point d : directions) {
            int nx = current.x + d.x;
            int ny = current.y + d.y;
            if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

            int c = getCost(gameState, nx, ny, playerId);
            if (c == Integer.MAX_VALUE) continue; // No se puede pasar por el rival.

            int newDist = dist[current.x][current.y] + c;
            if (newDist < dist[nx][ny]) {
                dist[nx][ny] = newDist;
                pq.add(new HexHeuristica.Node(nx, ny, newDist));
                //System.out.println("Actualizando casilla (" + nx + ", " + ny + ") con nueva distancia: " + newDist);
            }
        }
    }

    // No se encontró conexión.
    //System.out.println("No se encontró conexión para " + p);
    return 999999;
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



    private List<List<Point>> findAllPathsOptimized(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int[] nodes = getVirtualNodes(color, size);
        int startNode = nodes[0];
        int endNode = nodes[1];


        // Construir grafo
        List<List<Integer>> graph = buildGraphForPaths(s, color);


        // Calcular distMin con Dijkstra
        int distMin = calculateShortestDistanceDijkstra(graph, startNode, endNode);
        if (distMin == Integer.MAX_VALUE) {
            return new ArrayList<>();
        }
        int limitDist = distMin;


        // Filtrar grafo
        filterGraph(graph, startNode, endNode);


        List<List<Integer>> allPaths = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();
        List<Integer> currentPath = new ArrayList<>();


        findAllPathsRecursiveOptimized(graph, startNode, endNode, visited, currentPath, allPaths, limitDist, size);


        return convertPathsToPoints(allPaths, size);
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


   public double calculateAllPathsScoreOptimized(HexGameStatus s, PlayerType color) {
        List<List<Point>> paths = findAllPathsOptimized(s, color);
        double score = 0.0;
            score = paths.size();
        return score;
    }
     private int bloquearCaminoDelOponente(HexGameStatus gameState, PlayerType opponent) {
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if (distOpponent < 999999) {
            // Penalización inversamente proporcional a la distancia del oponente.
            // Cuanto menor sea la distancia, mayor la penalización.
            return -(1000 / (distOpponent + 1)); // +1 para evitar división por cero.
        }
        return 0;
    }

    
    private static final int[][] HEX_DIRECTIONS = {
    {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}
};

// Direcciones diagonales y sus adyacencias compartidas.
private static final int[][] DIAGONAL_DIRECTIONS = {
    {1, 1},  // Diagonal inferior derecha
    {-1, -1}, // Diagonal superior izquierda
    {1, -1},  // Diagonal inferior izquierda
    {-1, 1}   // Diagonal superior derecha
};

private static final int[][][] SHARED_ADJACENT_DIRECTIONS = {
    {{0, 1}, {1, 0}},  // Para la diagonal (1, 1)
    {{0, -1}, {-1, 0}}, // Para la diagonal (-1, -1)
    {{0, -1}, {1, 0}},  // Para la diagonal (1, -1)
    {{0, 1}, {-1, 0}}   // Para la diagonal (-1, 1)
};

/**
 * Construye un grafo basado en el estado del tablero y las restricciones del jugador.
 *
 * @param gameState Estado del tablero.
 * @param player Jugador para el cual se construye el grafo.
 * @return Lista de adyacencias representando el grafo.
 */
private List<List<Integer>> buildGraphForPaths(HexGameStatus s, PlayerType color) {
    int size = s.getSize();
    int totalNodes = size * size + 4;
    List<List<Integer>> graph = new ArrayList<>(totalNodes);
    for (int i = 0; i < totalNodes; i++) graph.add(new ArrayList<>());

    PlayerType opponent = getOpponentColor(color);
    int opponentVal = colorToInt(opponent);

    int P1_START = size * size;
    int P1_END = size * size + 1;
    int P2_START = size * size + 2;
    int P2_END = size * size + 3;

    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            int cellVal = s.getPos(x, y);
            if (cellVal != opponentVal) {
                int u = x * size + y;

                // Agregar conexiones hexagonales normales.
                for (int[] d : HEX_DIRECTIONS) {
                    int nx = x + d[0], ny = y + d[1];
                    if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                        int neighVal = s.getPos(nx, ny);
                        if (neighVal != opponentVal) {
                            int v = nx * size + ny;
                            graph.get(u).add(v);
                        }
                    }
                }

                // Agregar conexiones diagonales bajo la nueva regla.
                for (int i = 0; i < DIAGONAL_DIRECTIONS.length; i++) {
                    int[] diag = DIAGONAL_DIRECTIONS[i];
                    int nx = x + diag[0], ny = y + diag[1];

                    if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                        int diagVal = s.getPos(nx, ny);
                        if (diagVal != opponentVal) {
                            // Verificar las conexiones adyacentes compartidas.
                            int[] adj1 = SHARED_ADJACENT_DIRECTIONS[i][0];
                            int[] adj2 = SHARED_ADJACENT_DIRECTIONS[i][1];

                            int adj1x = x + adj1[0], adj1y = y + adj1[1];
                            int adj2x = x + adj2[0], adj2y = y + adj2[1];

                            boolean adj1Free = adj1x >= 0 && adj1x < size && adj1y >= 0 && adj1y < size
                                    && s.getPos(adj1x, adj1y) != opponentVal;
                            boolean adj2Free = adj2x >= 0 && adj2x < size && adj2y >= 0 && adj2y < size
                                    && s.getPos(adj2x, adj2y) != opponentVal;

                            if (adj1Free && adj2Free) {
                                int v = nx * size + ny;
                                graph.get(u).add(v);
                            }
                        }
                    }
                }
            }
        }
    }

    // Conexiones para PLAYER1.
    if (color == PlayerType.PLAYER1) {
        for (int yy = 0; yy < size; yy++) {
            if (s.getPos(0, yy) != opponentVal)
                graph.get(P1_START).add(0 * size + yy);
        }
        for (int yy = 0; yy < size; yy++) {
            if (s.getPos(size - 1, yy) != opponentVal)
                graph.get((size - 1) * size + yy).add(P1_END);
        }
    } else { // Conexiones para PLAYER2.
        for (int xx = 0; xx < size; xx++) {
            if (s.getPos(xx, 0) != opponentVal)
                graph.get(P2_START).add(xx * size + 0);
        }
        for (int xx = 0; xx < size; xx++) {
            if (s.getPos(xx, size - 1) != opponentVal)
                graph.get(xx * size + (size - 1)).add(P2_END);
        }
    }

    return graph;
}



    /**
     * Devuelve los nodos virtuales de inicio y fin para un jugador.
     */
    public int[] getVirtualNodes(PlayerType color, int size) {
        int P1_START = size * size;
        int P1_END = size * size + 1;
        int P2_START = size * size + 2;
        int P2_END = size * size + 3;
        if (color == PlayerType.PLAYER1) {
            return new int[]{P1_START, P1_END};
        } else {
            return new int[]{P2_START, P2_END};
        }
    }

    /**
     * Convierte el tipo de jugador a su ID numérico.
     */
    private int colorToInt(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? 1 : -1;
    }

    /**
     * Obtiene el color del oponente.
     */
    private PlayerType getOpponentColor(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
    }
    
    private List<Point> getTransitableTop(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) {
                fuentes.add(new Point(0, y));
            }
        }
        return fuentes;
    }
 private List<Point> getTransitableBottom(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(size - 1, y) != -playerId) {
                fuentes.add(new Point(size - 1, y));
            }
        }
        return fuentes;
    }


    /**
     * Recupera las celdas transitables desde la parte derecha del tablero para PLAYER2.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableRight(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, size - 1) != -playerId) {
                fuentes.add(new Point(x, size - 1));
            }
        }
        return fuentes;
    }
    private List<Point> getTransitableLeft(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, 0) != -playerId) {
                fuentes.add(new Point(x, 0));
            }
        }
        return fuentes;
    }
    

}
