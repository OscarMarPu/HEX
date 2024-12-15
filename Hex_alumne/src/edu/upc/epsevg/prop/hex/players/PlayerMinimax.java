package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.IPlayer;
import edu.upc.epsevg.prop.hex.IAuto;
import edu.upc.epsevg.prop.hex.PlayerMove;
import edu.upc.epsevg.prop.hex.PlayerType;
import edu.upc.epsevg.prop.hex.SearchType;
import java.awt.Point;
import java.util.ArrayList;
import java.util.List;

/**
 * Jugador que utiliza el algoritmo Minimax con poda alfa-beta.
 */
public class PlayerMinimax implements IPlayer, IAuto {

    private int profmax;            // Profundidad máxima
    private String name;            // Nombre del jugador
    private PlayerType myColor;     // Nuestro color (PLAYER1 o PLAYER2)
    private long nodesvisitats;     // Nodos visitados
    private long nodesestalviats;   // Nodos podados
    private static final int MAXIM = 100000000;
    private HexHeuristica heuristica;

    /**
     * Constructor que inicializa el jugador con la profundidad máxima.
     * 
     * @param name Nombre del jugador
     * @param prof Profundidad máxima de exploración
     */
    public PlayerMinimax(String name, int prof) {
        this.name = name;
        this.profmax = prof;
        this.heuristica = new HexHeuristica();
    }

    /**
     * Determina el mejor movimiento utilizando el algoritmo Minimax con poda alfa-beta.
     * 
     * @param s Estado actual del juego
     * @return Movimiento calculado
     */
    @Override
    public PlayerMove move(HexGameStatus s) {
        nodesvisitats = 0;
        nodesestalviats = 0;
        myColor = s.getCurrentPlayer();

        Point mejorMovimiento = null;
        int mejorValor = Integer.MIN_VALUE;
        int alpha = -MAXIM;
        int beta = MAXIM;

        List<Point> movimientosLegales = getLegalMoves(s);

        for (Point movimiento : movimientosLegales) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            int valor = minValor(nuevoEstado, alpha, beta, profmax - 1);
            if (valor > mejorValor) {
                mejorValor = valor;
                mejorMovimiento = movimiento;
            }
            alpha = Math.max(alpha, valor);
            if (beta <= alpha) {
                nodesestalviats++;
                break; // Poda alfa-beta
            }
        }

        /*System.out.println("Movimiento elegido: " + mejorMovimiento);
        System.out.println("Nodos visitados: " + nodesvisitats);
        System.out.println("Nodos podados: " + nodesestalviats);*/

        return new PlayerMove(mejorMovimiento, nodesvisitats, profmax, SearchType.MINIMAX);
    }

    /**
     * Retorna el nombre del jugador.
     * 
     * @return Nombre del jugador
     */
    @Override
    public String getName() {
        return "Minimax(" + name + ")";
    }
    
    @Override
    public void timeout() {
        // Implementación vacía, puedes añadir lógica si es necesario
        System.out.println("Timeout occurred.");
    }

    /**
     * Calcula el valor máximo utilizando Minimax con poda alfa-beta.
     * 
     * @param s Estado actual del juego
     * @param alpha Valor alfa actual
     * @param beta Valor beta actual
     * @param profundidad Profundidad restante
     * @return Valor máximo calculado
     */
    private int maxValor(HexGameStatus s, int alpha, int beta, int profundidad) {
        if (profundidad == 0 || s.isGameOver()) {
            return evaluarEstado(s);
        }

        int valor = Integer.MIN_VALUE;
        for (Point movimiento : getLegalMoves(s)) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            valor = Math.max(valor, minValor(nuevoEstado, alpha, beta, profundidad - 1));
            if (valor >= beta) {
                nodesestalviats++;
                return valor; // Poda beta
            }
            alpha = Math.max(alpha, valor);
        }
        return valor;
    }

    /**
     * Calcula el valor mínimo utilizando Minimax con poda alfa-beta.
     * 
     * @param s Estado actual del juego
     * @param alpha Valor alfa actual
     * @param beta Valor beta actual
     * @param profundidad Profundidad restante
     * @return Valor mínimo calculado
     */
    private int minValor(HexGameStatus s, int alpha, int beta, int profundidad) {
        if (profundidad == 0 || s.isGameOver()) {
            return evaluarEstado(s);
        }

        int valor = Integer.MAX_VALUE;
        for (Point movimiento : getLegalMoves(s)) {
            HexGameStatus nuevoEstado = applyMove(s, movimiento);
            valor = Math.min(valor, maxValor(nuevoEstado, alpha, beta, profundidad - 1));
            if (valor <= alpha) {
                nodesestalviats++;
                return valor; // Poda alfa
            }
            beta = Math.min(beta, valor);
        }
        return valor;
    }

    /**
     * Evalúa el estado actual del juego.
     * 
     * @param s Estado del juego
     * @return Valor heurístico del estado
     */
     private int evaluarEstado(HexGameStatus s) {
        nodesvisitats++;
        if (s.isGameOver()) {
            PlayerType ganador = s.GetWinner();
            return (ganador == myColor) ? MAXIM : -MAXIM;
        }
        return heuristica.evaluarEstado(s);
    }



    /**
     * Aplica un movimiento y devuelve el nuevo estado del juego.
     * 
     * @param s Estado actual del juego
     * @param movimiento Movimiento a aplicar
     * @return Nuevo estado del juego
     */
    private HexGameStatus applyMove(HexGameStatus s, Point movimiento) {
        HexGameStatus nuevoEstado = new HexGameStatus(s);
        nuevoEstado.placeStone(movimiento);
        return nuevoEstado;
    }

    /**
     * Obtiene los movimientos legales (celdas vacías).
     * 
     * @param s Estado actual del juego
     * @return Lista de movimientos legales
     */
    private List<Point> getLegalMoves(HexGameStatus s) {
        List<Point> movimientos = new ArrayList<>();
        int size = s.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (s.getPos(i, j) == 0) { // Celda vacía
                    movimientos.add(new Point(i, j));
                }
            }
        }
        return movimientos;
    }
}