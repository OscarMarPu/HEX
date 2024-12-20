package edu.upc.epsevg.prop.hex;

import edu.upc.epsevg.prop.hex.players.H_E_X_Player;
import edu.upc.epsevg.prop.hex.players.HumanPlayer;
import edu.upc.epsevg.prop.hex.players.PlayerMinimax;
import edu.upc.epsevg.prop.hex.players.RandomPlayer;



import javax.swing.SwingUtilities;

/**
 * Checkers: el joc de taula.
 * @author bernat
 */
public class Game {
        /**
     * @param args
     */
    public static void main(String[] args) { 
        
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                
                IPlayer player2 = new H_E_X_Player(2/*GB*/);
                //IPlayer player2 = new RandomPlayer("random");
                IPlayer player1 = new PlayerMinimax("OscarCabezas", 3);
                //IPlayer player1 = new HumanPlayer("Human1");
                //IPlayer player2 = new HumanPlayer("Human");
                                
                new Board(player1 , player2, 5/*mida*/,  10/*s*/, false);
             }
        });
    }
}
