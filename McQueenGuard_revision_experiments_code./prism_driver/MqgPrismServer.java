import java.io.*;
import parser.ast.*;
import prism.*;

/** Persistent (warm-JVM) PRISM verification server used by the McQueenGuard testbed.
 *  stdin : "<model.pm> <props.props>" per line   stdout: "OK <parse_ms> <build+check_ms> <states> v1 v2 ..." */
public class MqgPrismServer {
  public static void main(String[] args) throws Exception {
    ByteArrayOutputStream bos = new ByteArrayOutputStream();
    PrismLog log = new PrismPrintStreamLog(new PrintStream(bos, true));
    Prism prism = new Prism(log);
    prism.initialise();
    prism.setEngine(args.length>0 && args[0].equals("sparse") ? Prism.SPARSE : Prism.EXPLICIT);
    BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
    PrintStream out = new PrintStream(new FileOutputStream(FileDescriptor.out), true);
    String line;
    while ((line = br.readLine()) != null) {
      line = line.trim();
      if (line.isEmpty()) continue;
      if (line.equals("QUIT")) break;
      String[] p = line.split("\\s+");
      try {
        bos.reset();
        long t0 = System.nanoTime();
        ModulesFile mf = prism.parseModelFile(new File(p[0]));
        prism.loadPRISMModel(mf);
        PropertiesFile pf = prism.parsePropertiesFile(mf, new File(p[1]));
        long t1 = System.nanoTime();
        StringBuilder sb = new StringBuilder();
        String states = "?";
        for (int i = 0; i < pf.getNumProperties(); i++) {
          bos.reset();
          prism.modelCheck(pf, pf.getPropertyObject(i));
          log.flush();
          for (String l : bos.toString().split("\n")) {
            if (l.matches("^\\d+:\\(.*\\)=.*")) sb.append(l.substring(0, l.indexOf(':'))).append(':').append(l.substring(l.lastIndexOf('=') + 1).trim()).append(' ');
          }
          sb.append("| ");
        }
        long t2 = System.nanoTime();
        out.println("OK " + (t1 - t0) / 1e6 + " " + (t2 - t1) / 1e6 + " " + states + " " + sb.toString().trim());
      } catch (Exception e) {
        out.println("ERR " + e.getMessage().replace('\n', ' '));
      }
    }
  }
}
