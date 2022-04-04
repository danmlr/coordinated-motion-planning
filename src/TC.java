

import java.io.CharArrayReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.Reader;
import java.io.StringReader;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.channels.FileChannel.MapMode;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.Locale;
import java.util.NoSuchElementException;
import java.util.Scanner;
import java.util.regex.Pattern;

class TCException extends RuntimeException {
  private static final long serialVersionUID = 1L;

  TCException(String message) {
    super(message);
  }
}

/**
 * Cette classe regroupe les fonctions pratiques de lecture et d'écriture pour
 * le Tronc Commun.
 * <p>
 * Principe général : <br/>
 * Les fonctions de lecture utilisent une <b>entrée</b>. Par défaut, cette
 * entrée est <b>l'entrée standard</b>, c'est-à-dire l'entrée de la console qui
 * utilise normalement le clavier. Certaines fonctions de TC permettent de
 * rediriger cette entrée pour qu'elle utilise, par exemple, le contenu d'un
 * fichier à la place du clavier. <br/>
 * Les fonctions d'écriture utilisent une <b>sortie</b>. Par défaut, cette
 * sortie est <b>la sortie standard</b>, c'est-à-dire normalement l'affichage
 * dans la console. Certaines fonctions de TC permettent de rediriger cette
 * sortie pour qu'elle se fasse, par exemple, dans un fichier. Dans ce cas, tout
 * ce qu'on écrit en passant par les fonctions de sortie de TC, est enregistré
 * dans le fichier.
 * 
 * @version mai 2013
 * @author Francois Morain, Julien Cervelle, Philippe Chassignet, École
 *         Polytechnique
 * */
public class TC {

  private TC() {
  }

  private static final Object readMonitor = new Object();
  private static final Scanner IN = scanner(new InputStreamReader(System.in));
  private static volatile Scanner currentScanner = IN;
  private static final String NO_SUCH_ELEMENT = "La fin de l'entr\u00e9e est atteinte.";
  private static final String INPUT_MISMATCH = "L'entr\u00e9e n'est pas du type attendu.";

  private static Scanner scanner(Reader in) {
    Scanner scanner = new Scanner(in);
    scanner.useLocale(Locale.US);
    return scanner;
  }

  private static final Object writeMonitor = new Object();
  private static volatile PrintStream currentOutput = System.out;

  private static final Pattern DOT_PATTERN = Pattern.compile(".",
      Pattern.DOTALL);

  /**
   * Lecture du prochain caractère sur l'entrée. Tous les caractères, divers
   * blancs compris, sont pris en compte à tour de rôle. <br/>
   * Remarque : <code>finEntree()</code> renvoie <code>true</code> lorsqu'il ne
   * reste plus que des caractères blancs sur l'entrée.
   * 
   * @return le caractère lu, comme une valeur de type <code>char</code>
   * @throws TCException
   *           quand il n'y a plus de caractère sur l'entrée
   */
  public static char lireChar() {
    synchronized (readMonitor) {
      String match = currentScanner.findWithinHorizon(DOT_PATTERN, 1);
      if (match == null)
        throw new TCException(NO_SUCH_ELEMENT);
      return match.charAt(0);
    }
  }

  /**
   * Lecture d'un mot (une chaîne quelconque sans blanc) sur l'entrée.
   * 
   * @return le mot lu sous la forme d'une chaîne Java (<code>String</code>)
   * @throws TCException
   *           quand il n'y a plus de mot sur l'entrée
   */
  public static String lireMot() {
    try {
      synchronized (readMonitor) {
        return currentScanner.next();
      }
    } catch (NoSuchElementException e) {
      throw new TCException(NO_SUCH_ELEMENT);
    }
  }

  /**
   * Lecture d'un <tt>int</tt> sur l'entrée.
   * 
   * @return l'entier lu, comme une valeur de type <code>int</code>
   * @throws TCException
   *           quand le mot lu n'est pas un entier représentable dans le type
   *           <tt>int</tt> ou quand il n'y a plus de mot sur l'entrée
   */
  public static int lireInt() {
    try {
      synchronized (readMonitor) {
        return currentScanner.nextInt();
      }
    } catch (InputMismatchException e) {
      throw new TCException(INPUT_MISMATCH);
    } catch (NoSuchElementException e) {
      throw new TCException(NO_SUCH_ELEMENT);
    }
  }

  /**
   * Lecture d'un <tt>long</tt> sur l'entrée.
   * 
   * @return l'entier lu, comme une valeur de type <code>long</code>
   * @throws TCException
   *           quand le mot lu n'est pas un entier représentable dans le type
   *           <tt>long</tt> ou quand il n'y a plus de mot sur l'entrée
   */
  public static long lireLong() {
    try {
      synchronized (readMonitor) {
        return currentScanner.nextLong();
      }
    } catch (InputMismatchException e) {
      throw new TCException(INPUT_MISMATCH);
    } catch (NoSuchElementException e) {
      throw new TCException(NO_SUCH_ELEMENT);
    }

  }

  /**
   * Lecture d'un <tt>double</tt> sur l'entrée.
   * 
   * @return le nombre lu, comme une valeur de type <code>double</code>
   * @throws TCException
   *           quand le mot lu n'est pas un flottant représentable dans le type
   *           <tt>double</tt> ou quand il n'y a plus de mot sur l'entrée
   */
  public static double lireDouble() {
    try {
      synchronized (readMonitor) {
        return currentScanner.nextDouble();
      }
    } catch (InputMismatchException e) {
      throw new TCException(INPUT_MISMATCH);
    } catch (NoSuchElementException e) {
      throw new TCException(NO_SUCH_ELEMENT);
    }

  }

  /**
   * Lecture d'une ligne (tous les caractères, blancs compris, jusqu'au prochain
   * retour à la ligne) sur l'entrée. Le <i>retour à la ligne</i> est lu, mais
   * n'est pas dans la chaîne renvoyée.
   * 
   * @return la ligne lue sous la forme d'une chaîne Java (<code>String</code>)
   * @throws TCException
   *           quand il n'y a plus rien sur l'entrée
   */
  public static String lireLigne() {
    try {
      synchronized (readMonitor) {
        return currentScanner.nextLine();
      }
    } catch (NoSuchElementException e) {
      throw new TCException(NO_SUCH_ELEMENT);
    }

  }

  /**
   * Renvoie un tableau des mots obtenus en découpant le contenu d'une chaîne.
   * Plusieurs espaces (ou autres caractères blancs) consécutifs sont considérés
   * comme une seule séparation.
   * 
   * @param chaine
   *          la chaîne à découper
   * @return le tableau des mots obtenus
   */
  public static String[] motsDeChaine(String chaine) {
    if (chaine == null)
      throw new NullPointerException();
    return motsDeScanner(new Scanner(chaine));
  }

  private static String[] motsDeScanner(Scanner scanner) {
    ArrayList<String> list = new ArrayList<String>();
    while (scanner.hasNext())
      list.add(scanner.next());
    return list.toArray(new String[list.size()]);
  }

  /**
   * Renvoie un tableau de sous-chaînes obtenues en découpant le contenu d'une
   * chaîne selon un caractère séparateur. Un séparateur au début ou à la fin ou
   * plusieurs séparateurs consécutifs délimitent des sous-chaînes vides.
   * 
   * @param chaine
   *          la chaîne à découper
   * @param separateur
   *          le séparateur
   * @return le tableau des sous-chaînes obtenues
   */
  public static String[] decoupage(String chaine, char separateur) {
    String regEx = "" + separateur;
    if ("()$^*+?.[]\\{}|".indexOf(separateur) >= 0)
      regEx = "\\" + regEx;
    if (chaine == null)
      throw new NullPointerException();
    return chaine.split(regEx, -1);
  }

  /**
   * Teste s'il ne reste plus de mot (plus de caractère non blanc) sur l'entrée.
   * 
   * @return <tt>true</tt> s'il ne reste plus de mot (plus de caractère non
   *         blanc) sur l'entrée,<br/>
   *         <tt>false</tt> s'il reste au moins un mot (au moins un caractère
   *         non blanc) sur l'entrée.
   */
  public static boolean finEntree() {
    synchronized (readMonitor) {
      return !currentScanner.hasNext();
    }
  }

  /**
   * Redirige l'entrée pour que la lecture utilise le contenu d'un fichier.
   * 
   * @param nomFichier
   *          le nom du fichier
   */
  public static void lectureDansFichier(String nomFichier) {
    Scanner s;
    try {
      s = scanner(new FileReader(nomFichier));
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
    setScanner(s);
  }

  /**
   * Redirige l'entrée pour que la lecture utilise l'entrée standard de la
   * console (normalement le clavier). Cette fonction permet de revenir au mode
   * initial après avoir utilisé une autre entrée.
   */
  public static void lectureEntreeStandard() {
    setScanner(IN);
  }

  /**
   * Redirige l'entrée pour que la lecture utilise un <tt>Reader</tt>, pour
   * utilisateurs avertis.
   * 
   * @param input
   *          l'objet <tt>Reader</tt> utilisé comme entrée
   */
  public static void lectureReader(Reader input) {
    setScanner(scanner(input));
  }

  private static void setScanner(Scanner s) {
    synchronized (readMonitor) {
      if (currentScanner != IN)
        currentScanner.close();
      currentScanner = s;
    }
  }

  /**
   * Affiche un booléen sur la sortie.
   * 
   * @param b
   *          le booléen à afficher
   */
  public static void print(boolean b) {
    synchronized (writeMonitor) {
      currentOutput.print(b);
    }
  }

  /**
   * Affiche un caractère sur la sortie.
   * 
   * @param c
   *          le caractère à afficher
   */
  public static void print(char c) {
    synchronized (writeMonitor) {
      currentOutput.print(c);
    }
  }

  /**
   * Affiche un entier sur la sortie.
   * 
   * @param l
   *          l'entier (type <tt>long</tt>) à afficher
   */
  public static void print(long l) {
    synchronized (writeMonitor) {
      currentOutput.print(l);
    }
  }

  /**
   * Affiche un flottant sur la sortie.
   * 
   * @param d
   *          le flottant (type <tt>double</tt>) à afficher
   */
  public static void print(double d) {
    synchronized (writeMonitor) {
      currentOutput.print(d);
    }
  }

  /**
   * Affiche une chaîne sur la sortie.
   * 
   * @param chaine
   *          la chaîne à afficher
   */
  public static void print(String chaine) {
    synchronized (writeMonitor) {
      currentOutput.print(chaine);
    }
  }

  /**
   * Affiche le contenu d'un tableau de <tt>char</tt> sur la sortie.
   * 
   * @param t
   *          le tableau à afficher
   */
  public static void print(char[] t) {
    synchronized (writeMonitor) {
      for (char c : t)
        currentOutput.print(c);
    }
  }

  /**
   * Affiche un objet sur la sortie. <br/>
   * Cette fonction fait appel à la méthode <tt>o.toString()</tt> pour obtenir
   * une représentation de l'objet.
   * 
   * @param o
   *          une référence sur l'objet à afficher
   */
  public static void print(Object o) {
    synchronized (writeMonitor) {
      currentOutput.print(o);
    }
  }

  /** Va à la ligne sur l'entrée. */
  public static void println() {
    synchronized (writeMonitor) {
      currentOutput.println();
    }
  }

  /**
   * Affiche un booléen sur la sortie puis va à la ligne.
   * 
   * @param b
   *          le booléen à afficher
   */
  public static void println(boolean b) {
    synchronized (writeMonitor) {
      currentOutput.println(b);
    }
  }

  /**
   * Affiche un caractère sur la sortie puis va à la ligne.
   * 
   * @param c
   *          le caractère à afficher
   */
  public static void println(char c) {
    synchronized (writeMonitor) {
      currentOutput.println(c);
    }
  }

  /**
   * Affiche un entier sur la sortie puis va à la ligne.
   * 
   * @param l
   *          l'entier (type <tt>long</tt>) à afficher
   */
  public static void println(long l) {
    synchronized (writeMonitor) {
      currentOutput.println(l);
    }
  }

  /**
   * Affiche un flottant sur la sortie puis va à la ligne.
   * 
   * @param d
   *          le flottant (type <tt>double</tt>) à afficher
   */
  public static void println(double d) {
    synchronized (writeMonitor) {
      currentOutput.println(d);
    }
  }

  /**
   * Affiche une chaîne sur la sortie puis va à la ligne.
   * 
   * @param chaine
   *          la chaîne à afficher
   */
  public static void println(String chaine) {
    synchronized (writeMonitor) {
      currentOutput.println(chaine);
    }
  }

  /**
   * Affiche le contenu d'un tableau de <tt>char</tt> sur la sortie puis va à la
   * ligne.
   * 
   * @param t
   *          le tableau à afficher
   */
  public static void println(char[] t) {
    synchronized (writeMonitor) {
      for (char c : t)
        currentOutput.print(c);
      currentOutput.println();
    }
  }

  /**
   * Affiche un objet sur la sortie puis va à la ligne. <br/>
   * Cette fonction fait appel à la méthode <tt>o.toString()</tt> pour obtenir
   * une représentation de l'objet.
   * 
   * @param o
   *          une référence sur l'objet à afficher
   */
  public static void println(Object o) {
    synchronized (writeMonitor) {
      currentOutput.println(o);
    }
  }

  /**
   * Redirige la sortie des méthodes d'affichage pour écrire dans un
   * <i>nouveau</i> fichier. <br/>
   * S'il existe déjà un fichier avec ce nom, le contenu de ce fichier est
   * effacé pour commencer dans un fichier vide.
   * 
   * @param nomFichier
   *          le nom du fichier
   */
  public static void ecritureDansNouveauFichier(String nomFichier) {
    ecritureFichier(nomFichier, false);
  }

  /**
   * Redirige la sortie des méthodes d'affichage pour écrire à la fin d'un
   * fichier. <br/>
   * S'il n'existe pas de fichier avec ce nom, le fichier est créé pour
   * commencer dans un fichier vide. <br/>
   * S'il existe déjà un fichier avec ce nom, la sortie des méthodes d'affichage
   * est ajoutée à la fin du fichier.
   * 
   * @param nomFichier
   *          le nom du fichier
   */
  public static void ecritureEnFinDeFichier(String nomFichier) {
    ecritureFichier(nomFichier, true);
  }

  private static void ecritureFichier(String fileName, boolean append) {
    PrintStream output;
    try {
      output = new PrintStream(new FileOutputStream(fileName, append));
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
    ecriturePrintStream(output);
  }

  /**
   * Redirige la sortie des méthodes d'affichage vers la sortie standard,
   * c'est-à-dire normalement l'affichage dans la console. Cette fonction permet
   * de revenir au mode initial après avoir utilisé une autre sortie.
   */
  public static void ecritureSortieStandard() {
    // System.setOut(new PrintStream(System.out));
    ecriturePrintStream(System.out);
  }

  /**
   * Redirige la sortie des méthodes d'affichage vers un <tt>PrintStream</tt>,
   * pour utilisateurs avertis.
   * 
   * @param output
   *          l'objet <tt>PrintStream</tt> utilisé comme sortie
   */
  public static void ecriturePrintStream(PrintStream output) {
    synchronized (writeMonitor) {
      currentOutput.flush();
      if (currentOutput != System.out)
        currentOutput.close();
      currentOutput = output;
    }
  }

  // En dessous = ne sert plus en INF311
  // non visible hors package

  /**
   * Affiche un int sur la sortie.
   * 
   * @param i
   *          l'int à afficher
   */
  static void print(int i) {
    synchronized (writeMonitor) {
      currentOutput.print(i);
    }
  }

  /**
   * Affiche un int sur la sortie puis va à la ligne.
   * 
   * @param i
   *          le int à afficher
   */
  static void println(int i) {
    synchronized (writeMonitor) {
      currentOutput.println(i);
    }
  }

  /**
   * affiche un caractère sur la sortie
   * 
   * @param c
   *          le caractère à afficher
   */
  static void afficher(char c) {
    print(c);
  }

  /**
   * affiche un int sur la sortie
   * 
   * @param i
   *          l'int à afficher
   */
  static void afficher(int i) {
    print(i);
  }

  /**
   * affiche un long sur la sortie
   * 
   * @param l
   *          le long à afficher
   */
  static void afficher(long l) {
    print(l);
  }

  /**
   * affiche un double sur la sortie
   * 
   * @param d
   *          le double à afficher
   */
  static void afficher(double d) {
    print(d);
  }

  /**
   * affiche une chaîne sur la sortie
   * 
   * @param s
   *          la chaîne à afficher
   */
  static void afficher(String s) {
    print(s);
  }

  /**
   * affiche une chaîne passée comme un tableau de char sur la sortie
   * 
   * @param s
   *          la chaîne à afficher
   */
  static void afficher(char[] s) {
    print(s);
  }

  /** va à la ligne sur l'entrée */
  static void afficherln() {
    println();
  }

  /**
   * affiche un caractère sur la sortie
   * 
   * @param c
   *          le caractère à afficher
   */
  static void afficherln(char c) {
    println(c);
  }

  /**
   * affiche un int sur la sortie puis va à la ligne
   * 
   * @param i
   *          le int à afficher
   */
  static void afficherln(int i) {
    println(i);
  }

  /**
   * affiche un long sur la sortie puis va à la ligne
   * 
   * @param l
   *          le long à afficher
   */
  static void afficherln(long l) {
    println(l);
  }

  /**
   * affiche un double sur la sortie puis va à la ligne
   * 
   * @param d
   *          le double à afficher
   */
  static void afficherln(double d) {
    println(d);
  }

  /**
   * affiche une chaîne sur la sortie puis va à la ligne
   * 
   * @param s
   *          la chaîne à afficher
   */
  static void afficherlnString(String s) {
    println(s);
  }

  /**
   * affiche une chaîne passée comme un tableau de char sur la sortie puis va à
   * la ligne
   * 
   * @param s
   *          la chaîne à afficher
   */
  static void afficherlnTableauChar(char[] s) {
    println(s);
  }

  // en dessous = des vielleries
  // @Deprecated et non public

  /**
   * lecture d'un mot sur l'entrée (retourné sous forme de tableau de char)
   * 
   * @throws NoSuchElementException
   *           quand il n'y a plus d'entrée
   */
  @Deprecated
  static char[] lireMotSuivantTableauChar() {
    return lireMot().toCharArray();
  }

  /**
   * lecture d'une ligne sur l'entrée (retourné sous forme de tableau de char)
   * 
   * @throws NoSuchElementException
   *           quand il n'y a plus d'entrée
   */
  @Deprecated
  static char[] lireLigneTableauChar() {
    return lireLigne().toCharArray();
  }

  /**
   * Teste s'il reste des caractères non blancs sur l'entrée
   * 
   * @return true s'il reste des caractères non blancs sur l'entrée.
   */
  @Deprecated
  static boolean eof() {
    return finEntree();
  }

  /**
   * Teste s'il reste des caractères sur l'entrée
   * 
   * @return true s'il reste des caractères sur l'entrée.
   */
  @Deprecated
  static boolean fin() {
    synchronized (readMonitor) {
      return currentScanner.hasNextLine();
    }
  }

  /**
   * saute les blancs sur l'entrée (inutile pour les méthodes lire sauf
   * {@link #lireChar() lireChar} et {@link #lireLigne() lireLigne})
   */
  @Deprecated
  static void passerBlancs() {
    synchronized (readMonitor) {
      currentScanner.skip(currentScanner.delimiter());
    }
  }

  /**
   * Change l'entrée des méthodes lire pour que la lecture se fasse à partir
   * d'une chaîne
   * 
   * @param input
   *          la chaîne
   */
  static void lireDeString(String input) {
    synchronized (readMonitor) {
      currentScanner.close();
      currentScanner = scanner(new StringReader(input));
    }
  }

  /**
   * Change l'entrée des méthodes lire pour que la lecture se fasse à partir
   * d'une chaîne passée comme un tableau de char.
   * 
   * @param input
   *          la chaîne
   */
  static void lireDeChaine(char[] input) {
    synchronized (readMonitor) {
      currentScanner.close();
      currentScanner = scanner(new CharArrayReader(input));
    }
  }

  /**
   * Lit tous le contenu d'un fichier et le retourne sous forme de tableau de
   * char
   * 
   * @param fileName
   *          le nom de fichier
   * @return le contenu du fichier
   */
  static char[] charDeFichier(String fileName) {
    File file = new File(fileName);
    long size = file.length();
    ByteBuffer buffer;
    try {
      buffer = new FileInputStream(file).getChannel().map(MapMode.READ_ONLY, 0,
          size);
    } catch (IOException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
    CharBuffer charBuffer = Charset.defaultCharset().decode(buffer);
    char[] charArray = new char[charBuffer.limit()];
    charBuffer.get(charArray);
    return charArray;
  }

  /**
   * Lit tous le contenu d'un fichier et le retourne sous forme de chaîne java
   * 
   * @param fileName
   *          le nom de fichier
   * @return le contenu du fichier
   */
  static String chaineDeFichier(String fileName) {
    return new String(charDeFichier(fileName));
  }

  /**
   * Lit tous le contenu d'un fichier et le retourne sous forme de tableau de
   * char
   * 
   * @param fileName
   *          le nom de fichier
   * @return le contenu du fichier
   * @deprecated {@link #chaineDeFichier(String) chaineDeFichier} à la place (ce
   *             nom ne respecte pas les conventions de nommage qui stipule que
   *             les méthode doivent commencer par une minuscule)
   */
  @Deprecated
  static String StringDeFichier(String fileName) {
    return new String(charDeFichier(fileName));
  }

  /**
   * Lit tout le contenu d'une chaîne contenant des entier et les retourne
   * 
   * @param input
   *          la chaîne
   * @return le tableau des entiers lus
   * @throws InputMismatchException
   *           quand la chaîne contient autre chose que des entiers
   */
  static long[] longDeChaine(String input) {
    return longDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier contenant des entier et les retourne
   * 
   * @param fileName
   *          le nom de fichier
   * @return le tableau des entiers lus
   * @throws InputMismatchException
   *           quand le fichier contient autre chose que des entiers
   */
  static long[] longDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return longDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static long[] longDeScanner(Scanner scanner) {
    ArrayList<Long> list = new ArrayList<Long>();
    while (scanner.hasNext())
      list.add(scanner.nextLong());
    long[] ans = new long[list.size()];
    for (int i = 0; i < ans.length; i++)
      ans[i] = list.get(i);
    return ans;
  }

  /**
   * Lit tout le contenu d'une chaîne contenant des entier et les retourne
   * 
   * @param input
   *          la chaîne
   * @return le tableau des entiers lus
   * @throws InputMismatchException
   *           quand la chaîne contient autre chose que des entiers
   */
  static int[] intDeChaine(String input) {
    return intDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier contenant des entier et les retourne
   * 
   * @param fileName
   *          le nom de fichier
   * @return le tableau des entiers lus
   * @throws InputMismatchException
   *           quand le fichier contient autre chose que des entiers
   */
  static int[] intDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return intDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static int[] intDeScanner(Scanner scanner) {
    ArrayList<Integer> list = new ArrayList<Integer>();
    while (scanner.hasNext())
      list.add(scanner.nextInt());
    int[] ans = new int[list.size()];
    for (int i = 0; i < ans.length; i++)
      ans[i] = list.get(i);
    return ans;
  }

  /**
   * Lit tout le contenu d'une chaîne contenant des flottants et les retourne
   * 
   * @param input
   *          la chaîne
   * @return le tableau des flottants lus
   * @throws InputMismatchException
   *           quand la chaîne contient autre chose que des flottants
   */
  static double[] doubleDeChaine(String input) {
    return doubleDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier contenant des flottants et les retourne
   * 
   * @param fileName
   *          le nom de fichier
   * @return le tableau des flottants lus
   * @throws InputMismatchException
   *           quand le fichier contient autre chose que des flottants
   */
  static double[] doubleDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return doubleDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static double[] doubleDeScanner(Scanner scanner) {
    ArrayList<Double> list = new ArrayList<Double>();
    while (scanner.hasNext())
      list.add(scanner.nextDouble());
    double[] ans = new double[list.size()];
    for (int i = 0; i < ans.length; i++)
      ans[i] = list.get(i);
    return ans;
  }

  /**
   * Lit tout le contenu d'un fichier et retourne la tableau des mots
   * 
   * @param fileName
   *          le nom de fichier
   * @return le tableau des mots lus
   */
  static String[] motsDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return motsDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  /**
   * Lit tout le contenu d'une chaîne et retourne la tableau des mots sous forme
   * de tableau de caractères. Les tableaux peuvent donc \u00eatre de longueurs
   * distinctes.
   * 
   * @param input
   *          la chaîne
   * @return le tableau des mots lus
   */
  static char[][] motsTableauDeChaine(String input) {
    return motsTableauDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier et retourne la tableau des mots sous forme
   * de tableau de caractères. Les tableaux peuvent donc \u00eatre de longueurs
   * distinctes.
   * 
   * @param fileName
   *          le nom du fichier
   * @return le tableau des mots lus
   */
  static char[][] motsTableauDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return motsTableauDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static char[][] motsTableauDeScanner(Scanner scanner) {
    ArrayList<char[]> list = new ArrayList<char[]>();
    while (scanner.hasNext())
      list.add(scanner.next().toCharArray());
    return list.toArray(new char[list.size()][]);
  }

  /**
   * Lit tout le contenu d'une chaîne et retourne la tableau des lignes
   * 
   * @param input
   *          la chaîne
   * @return le tableau des lignes lues
   */
  static String[] lignesDeChaine(String input) {
    return lignesDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier et retourne la tableau des lignes
   * 
   * @param fileName
   *          le nom de fichier
   * @return le tableau des lignes lus
   */
  static String[] lignesDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return lignesDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static String[] lignesDeScanner(Scanner scanner) {
    ArrayList<String> list = new ArrayList<String>();
    while (scanner.hasNextLine())
      list.add(scanner.nextLine());
    return list.toArray(new String[list.size()]);
  }

  /**
   * Lit tout le contenu d'une chaîne et retourne la tableau des lignes sous
   * forme de tableau de caractères. Les tableaux peuvent donc \u00eatre de
   * longueurs distinctes.
   * 
   * @param input
   *          la chaîne
   * @return le tableau des lignes lus
   */
  static char[][] lignesTableauDeChaine(String input) {
    return lignesTableauDeScanner(new Scanner(input));
  }

  /**
   * Lit tout le contenu d'un fichier et retourne la tableau des lignes sous
   * forme de tableau de caractères. Les tableaux peuvent donc \u00eatre de
   * longueurs distinctes.
   * 
   * @param fileName
   *          le nom du fichier
   * @return le tableau des lignes lus
   */
  static char[][] lignesTableauDeFichier(String fileName) {
    Scanner scanner;
    try {
      scanner = new Scanner(new File(fileName));
      scanner.useLocale(Locale.US);
      return lignesTableauDeScanner(scanner);
    } catch (FileNotFoundException e) {
      throw new IllegalArgumentException(e.getMessage());
    }
  }

  private static char[][] lignesTableauDeScanner(Scanner scanner) {
    ArrayList<char[]> list = new ArrayList<char[]>();
    while (scanner.hasNextLine())
      list.add(scanner.nextLine().toCharArray());
    return list.toArray(new char[list.size()][]);
  }

  /**
   * Retourne le contenu d'une chaîne sous forme de tableau de char
   * 
   * @param input
   *          la chaîne
   * @return le contenu sous forme de tableau de char
   */
  static char[] charDeChaine(String input) {
    return input.toCharArray();
  }

  /**
   * Crée une nouvelle chaîne Java à partir d'un tableau de char
   * 
   * @param chaine
   *          le tableau
   * @return la chaîne de caractères Java
   */
  static String stringDeTableauChar(char[] chaine) {
    return new String(chaine);
  }

  private long nano = System.nanoTime();
  private long time = System.currentTimeMillis();

  /**
   * Démarre le chronomètre.
   * 
   * @see #tempsChrono()
   * @see #tempsNanoChrono()
   */
  void demarrerChrono() {
    time = System.currentTimeMillis();
    nano = System.nanoTime();
  }

  /**
   * Nombre de millisecondes écoulées depuis le démarrage du chronomètre.
   * 
   * @return le nombre de millisecondes écoulées depuis le démarrage du
   *         chronomètre
   */
  long tempsChrono() {
    return System.currentTimeMillis() - time;
  }

  /**
   * Nombre de nanosecondes écoulées depuis le démarrage du chronomètre.
   * 
   * @return le nambre de nanosecondes écoulées depuis le démarrage du
   *         chronomètre
   */
  long tempsNanoChrono() {
    return System.nanoTime() - nano;
  }
}
