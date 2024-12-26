package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.awt.Point;
import java.util.*;

public class Dijkstra {

    
    /**
     * Calcula el cost d'una cel·la en funció del seu estat i del jugador.
     * 
     * @param gameState Estat actual del joc.
     * @param x Coordenada X de la cel·la.
     * @param y Coordenada Y de la cel·la.
     * @param playerId Identificador del jugador.
     * @return El cost de la cel·la per al jugador.
     */
    public int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        return (cell == -playerId) ? Integer.MAX_VALUE : 1;
    }

    /**
     * Calcula la distància mínima des d'una font fins al destí mitjançant Dijkstra.
     * 
     * @param gameState Estat actual del joc.
     * @param p Tipus de jugador (PLAYER1 o PLAYER2).
     * @return La distància mínima.
     */
    public int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
        int size = gameState.getSize();
        int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;

        List<Point> fuentes = (p == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);

        if (fuentes.isEmpty()) return 999999;

        int[][] dist = new int[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        PriorityQueue<PlayerMinimax.Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

        for (Point f : fuentes) {
            int cost = getCost(gameState, f.x, f.y, playerId);
            if (cost != Integer.MAX_VALUE) {
                dist[f.x][f.y] = cost;
                pq.add(new PlayerMinimax.Node(f.x, f.y, cost));
            }
        }

        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        while (!pq.isEmpty()) {
            PlayerMinimax.Node current = pq.poll();

            if (current.distance > dist[current.x][current.y]) continue;

            if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
                (p == PlayerType.PLAYER2 && current.y == size - 1)) {
                return current.distance;
            }

            for (Point d : directions) {
                int nx = current.x + d.x;
                int ny = current.y + d.y;
                if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

                int c = getCost(gameState, nx, ny, playerId);
                if (c == Integer.MAX_VALUE) continue;

                int newDist = dist[current.x][current.y] + c;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    pq.add(new PlayerMinimax.Node(nx, ny, newDist));
                }
            }
        }

        return 999999;
    }


    /**
     * Calcula la distància més curta entre dos nodes en un graf utilitzant l'algorisme de Dijkstra.
     * 
     * @param graph Graf representat com una llista d'adjacències.
     * @param startNode Node inicial.
     * @param endNode Node final.
     * @return La distància més curta entre el node inicial i el node final.
     */
    public int calculateShortestDistanceDijkstra(List<List<Integer>> graph, int startNode, int endNode) {
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
    

    /**
     * Troba tots els camins entre dos nodes d'un graf de manera optimitzada.
     * 
     * @param graph Graf representat com una llista d'adjacències.
     * @param current Node actual.
     * @param endNode Node final.
     * @param visited Nodes visitats fins al moment.
     * @param currentPath Camí actual.
     * @param allPaths Llista de tots els camins trobats.
     * @param limitDist Límits de distància màxima.
     * @param size Mida del tauler.
     */
    public void findAllPathsRecursiveOptimized(List<List<Integer>> graph, int current, int endNode,
                                                Set<Integer> visited, List<Integer> currentPath,
                                                List<List<Integer>> allPaths, int limitDist, int size) {
        final int MAX_PATHS = 10000;
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


    /**
     * Troba tots els camins possibles en un tauler de joc Hex per a un jugador determinat.
     * 
     * @param s Estat del tauler de joc.
     * @param color Jugador per al qual es busquen els camins.
     * @return Llista de camins representats com a punts del tauler.
     */
    public List<List<Point>> findAllPathsOptimized(HexGameStatus s, PlayerType color) {
        int size = s.getSize();
        int[] nodes = getVirtualNodes(color, size);
        int startNode = nodes[0];
        int endNode = nodes[1];

        List<List<Integer>> graph = buildGraphForPaths(s, color);

        int distMin = calculateShortestDistanceDijkstra(graph, startNode, endNode);
        if (distMin == Integer.MAX_VALUE) {
            return new ArrayList<>();
        }
        int limitDist = distMin;

        filterGraph(graph, startNode, endNode);


        List<List<Integer>> allPaths = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();
        List<Integer> currentPath = new ArrayList<>();


        findAllPathsRecursiveOptimized(graph, startNode, endNode, visited, currentPath, allPaths, limitDist, size);


        return convertPathsToPoints(allPaths, size);
    }
    

    /**
     * Filtra el graf per mantenir només els nodes connectats entre els nodes inicial i final.
     * 
     * @param graph Graf a filtrar.
     * @param startNode Node inicial.
     * @param endNode Node final.
     */
    public void filterGraph(List<List<Integer>> graph, int startNode, int endNode) {
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


    /**
     * Construeix el graf invers d'un graf donat.
     * 
     * @param graph Graf original.
     * @return Graf invers.
     */
    public List<List<Integer>> buildReverseGraph(List<List<Integer>> graph) {
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
    
    /**
     * Obté tots els nodes accessibles des d'un node inicial mitjançant BFS.
     * 
     * @param g Graf representat com una llista d'adjacències.
     * @param start Node inicial.
     * @return Conjunt de nodes accessibles.
     */
    public Set<Integer> bfsReachable(List<List<Integer>> g,int start){
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
    
    /**
     * Converteix una llista de camins en format d'índex a format de punts del tauler.
     * 
     * @param paths Llista de camins com a índexs.
     * @param size Mida del tauler.
     * @return Llista de camins com a punts del tauler.
     */
    public List<List<Point>> convertPathsToPoints(List<List<Integer>> paths, int size) {
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

    /**
     * Calcula una puntuació basada en el nombre de camins trobats per a un jugador.
     * 
     * @param s Estat del tauler de joc.
     * @param color Jugador per al qual es calcula la puntuació.
     * @return Puntuació basada en el nombre de camins trobats.
     */
    public double calculateAllPathsScoreOptimized(HexGameStatus s, PlayerType color) {
        List<List<Point>> paths = findAllPathsOptimized(s, color);
        double score = 0.0;
            score = paths.size();
        return score;
    }
    
    public static final int[][] HEX_DIRECTIONS = {
        {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}
    };

    public static final int[][] DIAGONAL_DIRECTIONS = {
        {1, 1},
        {-1, -1},
        {1, -1}, 
        {-1, 1}   
    };

    public static final int[][][] SHARED_ADJACENT_DIRECTIONS = {
        {{0, 1}, {1, 0}}, 
        {{0, -1}, {-1, 0}},
        {{0, -1}, {1, 0}},
        {{0, 1}, {-1, 0}}
    };


    /**
     * Construeix un graf basat en l'estat del tauler i el jugador donat.
     * 
     * @param s Estat del joc.
     * @param color Jugador per al qual es construeix el graf.
     * @return Llista d'adjacències que representa el graf.
     */
    public List<List<Integer>> buildGraphForPaths(HexGameStatus s, PlayerType color) {
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

                    for (int i = 0; i < DIAGONAL_DIRECTIONS.length; i++) {
                        int[] diag = DIAGONAL_DIRECTIONS[i];
                        int nx = x + diag[0], ny = y + diag[1];

                        if (nx >= 0 && nx < size && ny >= 0 && ny < size) {
                            int diagVal = s.getPos(nx, ny);
                            if (diagVal != opponentVal) {
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

        if (color == PlayerType.PLAYER1) {
            for (int yy = 0; yy < size; yy++) {
                if (s.getPos(0, yy) != opponentVal)
                    graph.get(P1_START).add(0 * size + yy);
            }
            for (int yy = 0; yy < size; yy++) {
                if (s.getPos(size - 1, yy) != opponentVal)
                    graph.get((size - 1) * size + yy).add(P1_END);
            }
        } else {
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
     * Obté els nodes virtuals de començament i final per a un jugador.
     * 
     * @param color Jugador (PLAYER1 o PLAYER2).
     * @param size Mida del tauler.
     * @return Matriu amb els nodes virtuals d'inici i final.
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
     * Converteix el tipus de jugador en el seu identificador numèric.
     * 
     * @param color Jugador (PLAYER1 o PLAYER2).
     * @return Enter que representa el jugador (1 o -1).
     */
    public int colorToInt(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? 1 : -1;
    }

    /**
     * Obté el color de l'oponent d'un jugador.
     * 
     * @param color Jugador (PLAYER1 o PLAYER2).
     * @return El color de l'oponent.
     */
    public PlayerType getOpponentColor(PlayerType color) {
        return (color == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
    }

    /**
     * Obté les cel·les transitables des de la part superior del tauler per a PLAYER1.
     * 
     * @param gameState Estat actual del joc.
     * @param playerId Identificador del jugador.
     * @return Llista de punts transitables.
     */
    public List<Point> getTransitableTop(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) {
                fuentes.add(new Point(0, y));
            }
        }
        return fuentes;
    }

    /**
     * Obté les cel·les transitables des de l'esquerra del tauler per a PLAYER2.
     * 
     * @param gameState Estat actual del joc.
     * @param playerId Identificador del jugador.
     * @return Llista de punts transitables.
     */
    public List<Point> getTransitableLeft(HexGameStatus gameState, int playerId) {
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
